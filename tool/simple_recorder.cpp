/*
 * Copyright (c) 2014-2023, Yiliphoton Group
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-10     luhuadong    the first version
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>
#include <netinet/if_ether.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <netinet/ether.h>
#include <pcap.h>
#include <time.h>

#define PORT 51180
#define MAXLINE 9000

// #define SAVE_PATH "/mnt/sdcard/data/lidar"
#define SAVE_PATH "."

// #pragma pack(push, 1)
struct eth_header
{
    u_int8_t ether_dhost[ETH_ALEN]; /* destination eth addr     */
    u_int8_t ether_shost[ETH_ALEN]; /* source ether addr        */
    u_int16_t ether_type;           /* packet type ID field     */
} __attribute__((__packed__));

struct ip_header
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    unsigned int ihl : 4;
    unsigned int version : 4;
#elif __BYTE_ORDER == __BIG_ENDIAN
    unsigned int version : 4;
    unsigned int ihl : 4;
#else
#error "Please fix <bits/endian.h>"
#endif
    u_int8_t tos;
    u_int16_t tot_len;
    u_int16_t id;
    u_int16_t frag_off;
    u_int8_t ttl;
    u_int8_t protocol;
    u_int16_t check;
    u_int32_t saddr;
    u_int32_t daddr;
    /*The options start here. */
};
struct udp_header
{
    u_int16_t source;
    u_int16_t dest;
    u_int16_t len;
    u_int16_t check;
};
// #pragma pack(pop)

static int PacketNum = 0;

static pcap_t *PCAPFile;
static pcap_dumper_t *PCAPDump;
static bool flag = true;

static unsigned int count = 0;

static char *GetLocaltimeStr(char *buffer)
{
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);
    sprintf(buffer, "%d%02d%02d-%02d%02d%02d",
            (1900 + p->tm_year), (1 + p->tm_mon), p->tm_mday,
            p->tm_hour, p->tm_min, p->tm_sec);

    return buffer;
}

static int BuildEthernetIP4UDP(char *buffer, size_t totalLen, struct sockaddr_in from)
{
    /* Fill in ethernet header */
    struct eth_header *ethHeader = (struct eth_header *)buffer;
    struct ip_header *ipHeader = (struct ip_header *)(buffer + sizeof(struct eth_header));
    struct udp_header *udpHeader = (struct udp_header *)(buffer + sizeof(struct eth_header) + sizeof(struct ip_header));

    /* Ethernet II */
    unsigned char src_mac[6]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    unsigned char dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy((void *)ethHeader->ether_dhost, (void *)dest_mac, ETH_ALEN);
    memcpy((void *)ethHeader->ether_shost, (void *)src_mac, ETH_ALEN);
    ethHeader->ether_type = htons(ETH_P_IP);

    /* Internet Protocol */
    ipHeader->version = 4;
    ipHeader->ihl = 5;
    ipHeader->tos = 0;
    ipHeader->tot_len = htons(totalLen - sizeof(struct eth_header));
    ipHeader->id = 0;
    ipHeader->frag_off = htons(0x4000);
    ipHeader->ttl = 0xFF;
    ipHeader->protocol = 0x11;
    ipHeader->check = htons(0xFFFF);

    inet_pton(AF_INET, (char *)inet_ntoa(from.sin_addr), &(ipHeader->saddr));
    inet_pton(AF_INET, "255.255.255.0", &(ipHeader->daddr));

    /* UDP */
    udpHeader->len = htons(totalLen - sizeof(struct eth_header) - sizeof(struct ip_header));
    udpHeader->dest = htons(51180);
    udpHeader->source = from.sin_port;
    return 0;
}

static int WritePacket(struct pcap_pkthdr *packetHeader, const u_char *packetData)
{
    pcap_dump((u_char *)PCAPDump, packetHeader, packetData);
    return 0;
}

static void sigroutine(int signal)
{
    switch (signal)
    {
    case SIGINT:
        printf("Get a signal -- SIGINT\n");  // 2
        flag = false;
        break;
    default: break;
    }
    return;
}

// Driver code
int main(int argc, char *argv[])
{
    int sockfd;
    char buffer[MAXLINE];
    char *hello = "Hello from server";
    struct sockaddr_in servaddr, cliaddr;

    signal(SIGINT, sigroutine);

    if (argc > 1) {
        count = atoi(argv[1]);
    }

    char filename[128] = {0};
    char localtime[32] = {0};

    sprintf(filename, "%s/PointCloud-%s.pcap", SAVE_PATH, GetLocaltimeStr(localtime));

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    struct timeval tv;
    tv.tv_sec = 3;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Error: set socket recv timeout");
    }

    PCAPFile = pcap_open_dead(DLT_EN10MB, 65535);
    PCAPDump = pcap_dump_open(PCAPFile, filename);
    if (!PCAPDump)
    {
        printf("Error: %s\n", pcap_geterr(PCAPFile));
        pcap_close(PCAPFile);
        PCAPFile = 0;
        return -1;
    }

    printf("Establish UDP server (port: %d)\n", PORT);

    int len, n;

    len = sizeof(cliaddr); // len is value/result

    struct pcap_pkthdr header;
    const unsigned char *data = 0;
    unsigned int dataLength = 0;
    unsigned int dataHeaderLength = 0;
    double timeSinceStart = 0;
    int HeaderLen = sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct udp_header);

    while (flag)
    {
        char *msgbuf = buffer + HeaderLen;

        n = recvfrom(sockfd, (char *)msgbuf, MAXLINE, MSG_WAITALL,
                     (struct sockaddr *)&cliaddr, (socklen_t *)&len);

        if (n < 0) {
            perror("recvfrom() error");
            close(sockfd);
            continue;
        }
        msgbuf[n] = '\0';

        gettimeofday(&header.ts, NULL);

        printf("[%d] Client received %d bytes from %s\n", ++PacketNum, n, 
               (char *)inet_ntoa(cliaddr.sin_addr));

        BuildEthernetIP4UDP(buffer, n + HeaderLen, cliaddr);

        /* Save pcap */
        header.len = n + HeaderLen;
        header.caplen = n + HeaderLen;
        WritePacket(&header, (u_char *)buffer);
        count++;
    }

    printf("Save in '%s'\n", filename);
    // fclose(pcapfile);
    pcap_dump_close(PCAPDump);
    pcap_close(PCAPFile);

    return 0;
}