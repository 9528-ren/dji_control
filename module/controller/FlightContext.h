#pragma once
#include <stdint.h>

#include <string>

#include "TaskWorker.h"
#include "vehicle/VehicleBase.h"

namespace maicro {

/**
 * 飞行执行上下文(通用)。
 *
 * **为什么需要这层接口而不是让业务类各自持有 TaskWorker:**
 * 所有飞行动作(按键平移/返航/航迹/语音脚本)必须共用**同一个单槽执行器**
 * 与**同一套 generation 令牌**。这是"新指令抢占旧动作"的实现基础:
 * PostCmd 递增令牌并把进行中的脚本标记为中断, Execute 比对令牌丢弃过期命令。
 * 若各业务类各持一个 worker, 按键与脚本就会并发下发速度矢量 —— 那是
 * 飞行安全隐患, 不是解耦。
 *
 * 因此业务模块通过本接口复用执行器与少量共用飞行原语, 而不是复制它们。
 * 业务模块只负责自己的流程与参数, 不关心硬件细节。
 */
class FlightContext {
public:
  virtual ~FlightContext() {}

  /// 共享的单槽执行器(所有飞行动作串行化于此)
  virtual TaskWorker &Worker() = 0;

  /// 硬件抽象(业务层只经它访问飞机, 不直接碰 PSDK)
  virtual VehicleBase *Vehicle() = 0;

  /**
   * 自动起飞 + 可选续爬升。语音"准备"阶段复用此原语,
   * climbToTargetHeight=false 时只完成 SDK 自动起飞(约 1.2m), 不续爬。
   * controlObtained 回填是否已取得控制权, 供调用方决定是否需要释放。
   */
  virtual bool Takeoff(bool *controlObtained, bool climbToTargetHeight) = 0;

  /// 位置是否可作为可信参考点(排除无定位时的近原点垃圾坐标)
  virtual bool IsPositionSane(const S_MaicroPosition &pos) = 0;

  /// 当前飞行状态(单话题查询, 仅用于刹停前的兜底判断)
  virtual E_MaicroFlightStatus Status() = 0;

  /// 当前飞行模式(同上)
  virtual uint8_t DisplayMode() = 0;

  /// 向 Pilot 浮窗推送一条提示
  virtual void Notify(const std::string &msg) = 0;
};

}  // namespace maicro
