#pragma once

namespace asensing
{
namespace lidar
{

class SplitStrategyBySeq
{
public:

  SplitStrategyBySeq()
    : prev_frame_(0), prev_seq_(0)
  {
    setSafeRange();
  }

/*
这套“安全窗口”在做什么？

维护一个随 prev_seq_ 滑动的窗口 [safe_seq_min_, safe_seq_max_]。

期望下一包的 seq 落在这个窗口里（通常是 prev_seq_ 往前若干步，带一点冗余以容忍轻微乱序）。

一旦 seq 落到窗口左侧（seq < safe_seq_min_），被认为发生了回绕/归零，即进入下一帧，置 split=true。

如果 seq 落到窗口右侧（seq > safe_seq_max_），认为跳太远/大丢包，此处没有切帧，只是（可选）统计错误；若还在初始化阶段（prev_seq_==0），把当前包作为起点。

setSafeRange()（未贴出）通常会以 prev_seq_ 为中心，设置：

safe_seq_min_ = prev_seq_ - margin_left（经模运算处理回绕）

safe_seq_max_ = prev_seq_ + margin_right
或者依据每帧最大序号来计算一个“回绕阈值”。A2N 版本里传了 max_pkt_seq_，A2 版本没传，说明这个类内部可能用固定窗口宽度或协议常量来计算。

四种分支的直觉例子

设当前 prev_seq_=80，safe_seq_min_=70，safe_seq_max_=95：

A（切帧）：来了 seq=5 → 5 < 70 ⇒ 判定“新帧起点”，split=true。

B（小幅乱序）：来了 seq=78 → 70 ≤ 78 < 80 ⇒ 乱序但在安全区，不切帧，不更新 prev_seq_。

C（正常）：来了 seq=83 → 80 ≤ 83 ≤ 95 ⇒ 正常前进，更新 prev_seq_=83。

D（大跳）：来了 seq=140 → 140 > 95 ⇒ 视为严重丢包，不切帧（可记统计）。若这是初始化（prev_seq_==0），则用它作为起点。

设计取舍

优点：不用依赖帧号 frame 就能稳健切帧（很多雷达仅靠序号回绕来“到帧”）。对轻微乱序/网络抖动、个别丢包较鲁棒。

缺点/注意：

没用 frame 判定切帧：frame 只在正常路径更新保存；如果设备在换帧时 seq 不回绕，仅 frame 变了，这个函数不会因此切帧（依赖上层调用方在进入这个函数之前用 frame 触发）。你的 A2 版本中 split_strategy_.newPacket(frame_id, pkt_seq) 的调用侧就需要负责这一点。

窗口大小/边界很关键：safe_seq_min_/max_ 过窄会把正常包误判为丢包；过宽则降低切帧敏感度。初始化阶段也要小心（这里用 prev_seq_==0 作兜底）。

回绕判断：函数以“序号掉到下界左侧”为“回绕”。若协议的回绕点不是从 0 开始（例如最大序号 M 后回到 0），setSafeRange() 里必须做模环处理。
*/

  bool newPacket(uint32_t frame, uint16_t seq)
  {
    bool split = false;

    if (seq < safe_seq_min_) 
    {
      //// 情况A：本包序号落在安全下界左侧 → 认为上一帧结束、新帧开始
      prev_seq_ = seq;
      split = true;
    }
    else if (seq < prev_seq_) //min < seq < prev (The bag was switched in the range. no matter)
    {
      // do nothing.
      // 情况B：落在安全区间内但比上个序号小（可能乱序/重传/切换缓存）不更新 prev_seq_，也不切帧
    }
    else if (seq <= safe_seq_max_) //prev < seq < max (normal)
    {
      //// 情况C：正常前进（或持平）且仍在安全上界之内
      prev_seq_ = seq;
      prev_frame_ = frame;
    }
    else //max_ < seq (Serious packet loss)
    {
      if (prev_seq_ == 0) //initial
      {
        prev_seq_ = seq;
        prev_frame_ = frame;
      }

      //do nothing When it's lost, it's lost 
      //You can do packet loss statistics here
    }

    setSafeRange();
    return split;
  }

  bool newPacket(uint32_t frame, uint16_t seq, uint16_t max_seq)
  {
    bool split = false;

    if (seq < safe_seq_min_) 
    {      
      if( prev_seq_ < max_seq )
      {
        split = true;
        //std::cout << " 1: "  << " seq " << seq << " max_seq " << max_seq << " prev_seq_ " << prev_seq_ << " safe_seq_max_ " << safe_seq_max_ <<        " safe_seq_min_ " << safe_seq_min_ << std::endl;
      }
      else
      {
        split = false; 
      }
      prev_seq_ = seq;
    }
    else if (seq < prev_seq_) //min < seq < prev (The bag was switched in the range. no matter)
    {
      // do nothing.
    }
    else if (seq <= safe_seq_max_) //prev < seq < max (normal)
    {      
      if(seq >= max_seq)
      {
        split = true;
        //std::cout << " 2: " << " seq " << seq << " max_seq " << max_seq << " prev_seq_ " << prev_seq_ << " safe_seq_max_ " << safe_seq_max_ <<        " safe_seq_min_ " << safe_seq_min_ << std::endl;
      }
      prev_seq_ = seq;
      prev_frame_ = frame;
    }
    else //max_ < seq (Serious packet loss)
    {
      if (prev_seq_ == 0) //initial
      {
        prev_seq_ = seq;
        prev_frame_ = frame;
      }

      if(seq >= max_seq)
      {
        //std::cout << " 3: "  << " seq " << seq << " max_seq " << max_seq << " prev_seq_ " << prev_seq_ << " safe_seq_max_ " << safe_seq_max_ <<        " safe_seq_min_ " << safe_seq_min_ << std::endl;
        prev_seq_ = seq;
        prev_frame_ = frame;
        split = true;
      }
      //do nothing When it's lost, it's lost 
      //You can do packet loss statistics here
    }

    setSafeRange();    
      
    return split;
  }

  bool newPacket(uint32_t frame)
  {
    bool split = false;
    if (frame != prev_frame_)
    {
      split = true;
      prev_frame_ = frame;
      prev_seq_ = 0;  // 可选：重置序号
    }
    setSafeRange();
    return split;
  }

#ifndef UNIT_TEST
private:
#endif

  constexpr static uint16_t RANGE = 10;

  void setSafeRange()
  {
    safe_seq_min_ = (prev_seq_ > RANGE) ? (prev_seq_ - RANGE) : 0;
    safe_seq_max_ = prev_seq_ + RANGE;
  }

  uint32_t prev_frame_;
  uint16_t prev_seq_;
  uint16_t safe_seq_min_;
  uint16_t safe_seq_max_;
};

}  // namespace lidar
}  // namespace asensing
