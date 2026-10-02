#pragma once

// ════════════════════════════════════════════════════════════════════
//  结构化错误码（协议域，单一来源）
//  ────────────────────────────────────────────────────────────────────
//  reply / event 携带的机器可读错误码：调用方（前端 / 未来 agent / 运维）
//  按码分支，而不是解析人类可读的 message。message 仍保留。
//
//  分组区间：
//    0        成功
//    1xxx     协议 / 路由类（bridge 产生）
//    2xxx     领域 / 状态类（子模块产生；P1 下游失败统一归 2000）
//    9xxx     内部错误
//  依赖：无
// ════════════════════════════════════════════════════════════════════

#include <cstdint>

namespace RusUtils {

    /// 统一错误码（协议 v0.5）
    enum class ErrorCode : uint32_t {
        kOk = 0,                    // 成功（error_code 缺省值）

        // ── 1xxx 协议 / 路由（bridge 产生）──
        kUnknownCommand     = 1001, // 未注册 / 未知指令
        kInvalidArgs        = 1002, // 参数校验失败
        kMalformedMessage   = 1003, // 报文格式错误
        kTimeout            = 1004, // 下游服务调用超时
        kServiceUnavailable = 1005, // 下游服务未就绪
        kServiceException   = 1006, // 下游服务调用异常

        // ── 2xxx 领域 / 状态（子模块产生）──
        kModuleFailure      = 2000, // 通用领域失败（P1 下游统一归此）
        kNotReady           = 2001, // 前置条件不满足（未预扫查 / 无轨迹等）
        kBusy               = 2002, // 已在执行中
        kNoData             = 2003, // 无点云 / 无文件等
        kNotConnected       = 2004, // 未连接
        kPlanFailed         = 2005, // 轨迹生成 / 插值失败
        kMotionFailed       = 2006, // 运动 / 执行失败
        kIoError            = 2007, // 落盘 / 读盘错误
        kUnsupported        = 2008, // 已定义但未实现

        // ── 9xxx 内部 ──
        kInternal           = 9001, // 内部错误
    };

    /// 错误码 → 人类可读名（日志 / 调试）
    inline const char* error_code_name(ErrorCode c) {
        switch (c) {
            case ErrorCode::kOk:                 return "OK";
            case ErrorCode::kUnknownCommand:     return "UNKNOWN_COMMAND";
            case ErrorCode::kInvalidArgs:        return "INVALID_ARGS";
            case ErrorCode::kMalformedMessage:   return "MALFORMED_MESSAGE";
            case ErrorCode::kTimeout:            return "TIMEOUT";
            case ErrorCode::kServiceUnavailable: return "SERVICE_UNAVAILABLE";
            case ErrorCode::kServiceException:   return "SERVICE_EXCEPTION";
            case ErrorCode::kModuleFailure:      return "MODULE_FAILURE";
            case ErrorCode::kNotReady:           return "NOT_READY";
            case ErrorCode::kBusy:               return "BUSY";
            case ErrorCode::kNoData:             return "NO_DATA";
            case ErrorCode::kNotConnected:       return "NOT_CONNECTED";
            case ErrorCode::kPlanFailed:         return "PLAN_FAILED";
            case ErrorCode::kMotionFailed:       return "MOTION_FAILED";
            case ErrorCode::kIoError:            return "IO_ERROR";
            case ErrorCode::kUnsupported:        return "UNSUPPORTED";
            case ErrorCode::kInternal:           return "INTERNAL";
        }
        return "UNKNOWN";
    }

    inline constexpr uint32_t to_u32(ErrorCode c) { return static_cast<uint32_t>(c); }

}  // namespace RusUtils
