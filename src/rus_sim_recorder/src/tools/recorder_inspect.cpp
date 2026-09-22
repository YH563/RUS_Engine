// ════════════════════════════════════════════════════════════════════
//  记录文件体检工具（rus_sim_recorder_inspect）
//  ────────────────────────────────────────────────────────────────────
//  读 .rusrec：打印头部 / 通道表 / 逐通道统计（记录数、吞吐、时间跨度、频率、
//  seq 跳变、CRC 失败），并校验"尾索引 ↔ 实际内容"一致性。
//
//  两条读取路径：
//    正常文件  读尾索引 → 免扫描给出统计（快），并做索引自洽性校验
//    --scan    顺序扫描全部记录（崩溃文件【无尾索引】自动降级到这条路径）
//    --check-crc 顺序扫描并逐条校验 payload CRC32（隐含 --scan）
//
//  退出码：0 = 完好；1 = 打不开 / 头部非法；2 = 能读但有问题（CRC / 截断 / 索引不一致）
//
//  用法：
//    ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_20260922_141530.rusrec
//    ros2 run rus_sim_recorder rus_sim_recorder_inspect <file> --scan --dump 5
//  也可直接执行 install/rus_sim_recorder/lib/rus_sim_recorder/rus_sim_recorder_inspect（无需 ROS）
// ════════════════════════════════════════════════════════════════════

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

#include "storage/rec_reader.hpp"

using RusRecorder::Components::ChannelDesc;
using RusRecorder::Components::kFooterSize;
using RusRecorder::Components::RecHeader;
using RusRecorder::Storage::RecReader;

namespace {

    constexpr double kMiB = 1024.0 * 1024.0;

    /// unix ns → 本地时间字符串（毫秒精度；≤0 视为无时间戳）
    std::string FormatNs(int64_t ns)
    {
        if (ns <= 0) return "—";
        const std::time_t sec = static_cast<std::time_t>(ns / 1000000000LL);
        const long ms = static_cast<long>((ns % 1000000000LL) / 1000000LL);
        std::tm tm{};
        localtime_r(&sec, &tm);
        char buf[64] = {0};
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
        char out[80] = {0};
        std::snprintf(out, sizeof(out), "%s.%03ld", buf, ms);
        return out;
    }

    /// 记录频率（Hz）：按消息时间戳跨度算
    double RateHz(uint64_t records, int64_t first_ns, int64_t last_ns)
    {
        if (records < 2 || last_ns <= first_ns) return 0.0;
        const double span = static_cast<double>(last_ns - first_ns) / 1e9;
        return span > 0.0 ? static_cast<double>(records - 1) / span : 0.0;
    }

    std::string JsonEscape(const std::string& s)
    {
        std::string out;
        out.reserve(s.size() + 8);
        for (char c : s) {
            switch (c) {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        char b[8];
                        std::snprintf(b, sizeof(b), "\\u%04x", static_cast<unsigned char>(c));
                        out += b;
                    } else {
                        out += c;
                    }
            }
        }
        return out;
    }

    void PrintUsage(const char* exe)
    {
        std::printf(
            "用法：%s <file.rusrec> [选项]\n"
            "选项：\n"
            "  --scan        顺序扫描全部记录（无尾索引的崩溃文件自动走这条路径）\n"
            "  --check-crc   顺序扫描并逐条校验 payload CRC32（隐含 --scan）\n"
            "  --dump N      额外打印前 N 条记录明细（默认 0）\n"
            "  --json        以 JSON 输出（脚本消费；退出码含义不变）\n"
            "  -h, --help    显示本帮助\n"
            "退出码：0=完好，1=打不开/头部非法，2=能读但有问题（CRC/截断/索引不一致）\n",
            exe);
    }

    /// 每通道统计行（扫描结果优先；未扫描时用尾索引聚合，等价信息）
    struct ChannelRow {
        uint64_t records = 0;
        uint64_t payload_bytes = 0;
        uint32_t max_payload_size = 0;
        uint64_t seq_gaps = 0;
        uint64_t corrupt = 0;
        int64_t first_stamp_ns = 0;
        int64_t last_stamp_ns = 0;
    };

    std::vector<ChannelRow> BuildRows(const RecReader& reader, bool scanned)
    {
        std::vector<ChannelRow> rows(reader.channels().size());
        if (scanned) {
            const auto& st = reader.channel_stats();
            for (size_t i = 0; i < rows.size() && i < st.size(); ++i) {
                rows[i].records = st[i].records;
                rows[i].payload_bytes = st[i].payload_bytes;
                rows[i].max_payload_size = st[i].max_payload_size;
                rows[i].seq_gaps = st[i].seq_gaps;
                rows[i].corrupt = st[i].corrupt;
                rows[i].first_stamp_ns = st[i].first_stamp_ns;
                rows[i].last_stamp_ns = st[i].last_stamp_ns;
            }
            return rows;
        }

        std::map<uint16_t, size_t> idx;
        for (size_t i = 0; i < reader.channels().size(); ++i) {
            idx[reader.channels()[i].channel_id] = i;
        }
        for (const auto& e : reader.index()) {
            const auto it = idx.find(e.channel_id);
            if (it == idx.end()) continue;
            ChannelRow& r = rows[it->second];
            if (r.records == 0) r.first_stamp_ns = e.stamp_ns;
            r.last_stamp_ns = e.stamp_ns;
            r.records++;
            r.payload_bytes += e.payload_size;
            r.max_payload_size = std::max(r.max_payload_size, e.payload_size);
        }
        return rows;
    }

    void PrintText(const std::string& path, const RecReader& reader, bool scanned, bool check_crc,
                   int64_t trailing, const std::vector<std::string>& problems,
                   const std::vector<std::string>& notes, const std::vector<RecHeader>& dump)
    {
        const auto& hdr = reader.header();
        const auto& channels = reader.channels();
        const std::vector<ChannelRow> rows = BuildRows(reader, scanned);

        uint64_t total_records = 0;
        uint64_t total_bytes = 0;
        int64_t first_ns = 0;
        int64_t last_ns = 0;
        for (const auto& r : rows) {
            total_records += r.records;
            total_bytes += r.payload_bytes;
            if (r.records > 0) {
                if (first_ns == 0 || (r.first_stamp_ns > 0 && r.first_stamp_ns < first_ns)) {
                    first_ns = r.first_stamp_ns;
                }
                if (r.last_stamp_ns > last_ns) last_ns = r.last_stamp_ns;
            }
        }
        const bool use_index_stats = !scanned;

        std::printf("═══ 文件 ═══\n");
        std::printf("  路径      %s\n", path.c_str());
        std::printf("  大小      %.2f MiB（%lld B）\n",
                    static_cast<double>(reader.file_size()) / kMiB,
                    static_cast<long long>(reader.file_size()));
        std::printf("  格式      v%u（file header %u / rec header %u / index %u / footer %u 字节）\n",
                    hdr.version, hdr.header_size, hdr.rec_header_size, hdr.index_entry_size,
                    static_cast<unsigned>(kFooterSize));
        std::printf("  创建      %s\n", FormatNs(hdr.created_unix_ns).c_str());
        std::printf("  尾索引    %s\n", reader.has_index()
                    ? ("有（" + std::to_string(reader.index().size()) + " 项，" +
                       (reader.index_sane() ? "自洽" : "不自洽") + "）").c_str()
                    : "无（文件未正常关闭）");

        std::printf("\n═══ 通道 ═══\n");
        std::printf("  %-4s %-8s %-22s %-40s %s\n", "id", "kind", "topic", "type", "说明");
        for (const auto& ch : channels) {
            std::printf("  %-4u %-8s %-22s %-40s %s\n", ch.channel_id,
                        RusRecorder::Components::payload_kind_name(ch.kind), ch.topic.c_str(),
                        ch.type_name.c_str(), ch.note.c_str());
        }

        std::printf("\n═══ 统计（%s）═══\n",
                    use_index_stats ? "尾索引聚合，未读 payload（如需逐条校验加 --check-crc）"
                                    : (check_crc ? "顺序扫描 + CRC32 校验" : "顺序扫描"));
        std::printf("  %-4s %10s %14s %10s %10s %9s %9s %8s %8s\n", "通道", "记录数", "payload",
                    "平均", "最大", "时长(s)", "频率(Hz)", "seq跳变", "CRC失败");
        for (size_t i = 0; i < rows.size(); ++i) {
            const ChannelRow& r = rows[i];
            const double span = (r.records > 1 && r.last_stamp_ns > r.first_stamp_ns)
                ? static_cast<double>(r.last_stamp_ns - r.first_stamp_ns) / 1e9
                : 0.0;
            const double avg = r.records > 0
                ? static_cast<double>(r.payload_bytes) / static_cast<double>(r.records) : 0.0;
            std::printf("  %-4u %10llu %11.2f MiB %7.1f KiB %7.1f KiB %9.2f %9.1f %8llu %8llu\n",
                        channels[i].channel_id,
                        static_cast<unsigned long long>(r.records),
                        static_cast<double>(r.payload_bytes) / kMiB, avg / 1024.0,
                        static_cast<double>(r.max_payload_size) / 1024.0, span,
                        RateHz(r.records, r.first_stamp_ns, r.last_stamp_ns),
                        static_cast<unsigned long long>(r.seq_gaps),
                        static_cast<unsigned long long>(r.corrupt));
        }
        std::printf("  合计      %llu 条 / %.2f MiB（payload）\n",
                    static_cast<unsigned long long>(total_records),
                    static_cast<double>(total_bytes) / kMiB);
        std::printf("  时间范围  %s  →  %s\n", FormatNs(first_ns).c_str(), FormatNs(last_ns).c_str());
        if (first_ns > 0 && last_ns > first_ns) {
            std::printf("  跨度      %.2f s\n",
                        static_cast<double>(last_ns - first_ns) / 1e9);
        }
        if (scanned) {
            std::printf("  扫描覆盖  %llu B；尾部残留 %lld B%s\n",
                        static_cast<unsigned long long>(reader.scan_end_offset()),
                        static_cast<long long>(trailing),
                        reader.has_index() ? "（正常文件应恰为 索引区 + Footer）" : "");
        }

        if (!dump.empty()) {
            std::printf("\n═══ 前 %zu 条记录 ═══\n", dump.size());
            std::printf("  %-6s %-5s %-8s %8s  %-24s %10s  %s\n",
                        "序号", "通道", "kind", "seq", "时间戳", "大小", "CRC32");
            for (size_t i = 0; i < dump.size(); ++i) {
                const RecHeader& rh = dump[i];
                std::printf("  %-6zu %-5u %-8s %8u  %-24s %7u B  0x%08X\n", i, rh.channel_id,
                            RusRecorder::Components::payload_kind_name(rh.kind), rh.seq,
                            FormatNs(rh.stamp_ns).c_str(), rh.payload_size, rh.payload_crc32);
            }
        }

        std::printf("\n═══ 结论 ═══\n");
        if (problems.empty()) {
            std::printf("  ✔ 文件完好：结构一致，未发现 CRC 失败 / 截断\n");
        } else {
            std::printf("  ✘ 发现问题 %zu 处：\n", problems.size());
            for (const auto& p : problems) std::printf("    · %s\n", p.c_str());
        }
        for (const auto& n : notes) std::printf("  · %s\n", n.c_str());
    }

    void PrintJson(const std::string& path, const RecReader& reader, bool scanned, bool check_crc,
                   int64_t trailing, const std::vector<std::string>& problems,
                   const std::vector<std::string>& notes)
    {
        const auto& hdr = reader.header();
        const std::vector<ChannelRow> rows = BuildRows(reader, scanned);

        uint64_t total_records = 0;
        uint64_t total_bytes = 0;
        for (const auto& r : rows) {
            total_records += r.records;
            total_bytes += r.payload_bytes;
        }

        std::printf("{\"file\":\"%s\"", JsonEscape(path).c_str());
        std::printf(",\"size_bytes\":%lld", static_cast<long long>(reader.file_size()));
        std::printf(",\"format_version\":%u", hdr.version);
        std::printf(",\"created_unix_ns\":%lld", static_cast<long long>(hdr.created_unix_ns));
        std::printf(",\"has_index\":%s", reader.has_index() ? "true" : "false");
        std::printf(",\"index_sane\":%s", reader.index_sane() ? "true" : "false");
        std::printf(",\"scanned\":%s", scanned ? "true" : "false");
        std::printf(",\"crc_checked\":%s", (scanned && check_crc) ? "true" : "false");
        std::printf(",\"records\":%llu", static_cast<unsigned long long>(total_records));
        std::printf(",\"payload_bytes\":%llu", static_cast<unsigned long long>(total_bytes));
        std::printf(",\"corrupt_records\":%llu",
                    static_cast<unsigned long long>(scanned ? reader.corrupt_records() : 0));
        std::printf(",\"trailing_bytes\":%lld", static_cast<long long>(trailing));
        std::printf(",\"healthy\":%s", problems.empty() ? "true" : "false");

        std::printf(",\"channels\":[");
        for (size_t i = 0; i < reader.channels().size(); ++i) {
            const auto& ch = reader.channels()[i];
            const ChannelRow& r = rows[i];
            std::printf("%s{\"id\":%u,\"kind\":\"%s\",\"topic\":\"%s\",\"type\":\"%s\",\"note\":\"%s\"",
                        i ? "," : "", ch.channel_id,
                        RusRecorder::Components::payload_kind_name(ch.kind),
                        JsonEscape(ch.topic).c_str(), JsonEscape(ch.type_name).c_str(),
                        JsonEscape(ch.note).c_str());
            std::printf(",\"records\":%llu,\"payload_bytes\":%llu,\"max_payload_size\":%u",
                        static_cast<unsigned long long>(r.records),
                        static_cast<unsigned long long>(r.payload_bytes), r.max_payload_size);
            std::printf(",\"first_stamp_ns\":%lld,\"last_stamp_ns\":%lld",
                        static_cast<long long>(r.first_stamp_ns),
                        static_cast<long long>(r.last_stamp_ns));
            std::printf(",\"seq_gaps\":%llu,\"corrupt\":%llu,\"rate_hz\":%.3f}",
                        static_cast<unsigned long long>(r.seq_gaps),
                        static_cast<unsigned long long>(r.corrupt),
                        RateHz(r.records, r.first_stamp_ns, r.last_stamp_ns));
        }
        std::printf("]");

        std::printf(",\"problems\":[");
        for (size_t i = 0; i < problems.size(); ++i) {
            std::printf("%s\"%s\"", i ? "," : "", JsonEscape(problems[i]).c_str());
        }
        std::printf("],\"notes\":[");
        for (size_t i = 0; i < notes.size(); ++i) {
            std::printf("%s\"%s\"", i ? "," : "", JsonEscape(notes[i]).c_str());
        }
        std::printf("]}\n");
    }

}  // namespace

int main(int argc, char** argv)
{
    std::string path;
    bool force_scan = false;
    bool check_crc = false;
    bool json = false;
    int dump_n = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else if (a == "--scan") {
            force_scan = true;
        } else if (a == "--check-crc") {
            check_crc = true;
            force_scan = true;
        } else if (a == "--json") {
            json = true;
        } else if (a == "--dump") {
            if (i + 1 >= argc) {
                std::printf("--dump 需要参数\n");
                return 1;
            }
            dump_n = std::atoi(argv[++i]);
        } else if (!a.empty() && a[0] == '-') {
            std::printf("未知选项：%s\n", a.c_str());
            PrintUsage(argv[0]);
            return 1;
        } else if (path.empty()) {
            path = a;
        } else {
            std::printf("多余参数：%s\n", a.c_str());
            return 1;
        }
    }
    if (path.empty()) {
        PrintUsage(argv[0]);
        return 1;
    }

    RecReader reader;
    std::string err;
    if (!reader.Open(path, &err)) {
        std::printf("打开失败：%s\n", err.c_str());
        return 1;
    }

    // 崩溃文件（无尾索引）必须扫描；显式 --scan / --check-crc 也走扫描路径
    const bool scanned = force_scan || !reader.has_index();
    std::vector<RecHeader> dump;
    std::string scan_err;
    if (scanned) {
        reader.Scan([&](const RecHeader& rh, const uint8_t*) {
            if (static_cast<int>(dump.size()) < dump_n) dump.push_back(rh);
            return true;
        }, check_crc, &scan_err);
    }

    const int64_t trailing = scanned
        ? reader.file_size() - static_cast<int64_t>(reader.scan_end_offset())
        : 0;
    const int64_t index_bytes_expected = reader.has_index()
        ? static_cast<int64_t>(reader.index().size()) * reader.header().index_entry_size +
              kFooterSize
        : 0;

    // ── 判定：可疑点汇总（problems 空 = 完好）──
    std::vector<std::string> problems;
    std::vector<std::string> notes;
    if (!scan_err.empty()) problems.push_back(scan_err);
    if (!reader.has_index()) {
        notes.push_back("文件未正常关闭（无尾索引）：记录可顺序读回，但无法随机定位");
    } else if (!reader.index_sane()) {
        problems.push_back("尾索引不自洽（offset 非单调 / 与文件尺寸不符）");
    }
    if (reader.has_index() && scanned) {
        if (reader.scanned_records() != reader.index().size()) {
            problems.push_back("尾索引条数（" + std::to_string(reader.index().size()) +
                               "）与实际记录数（" + std::to_string(reader.scanned_records()) +
                               "）不一致");
        } else if (trailing != index_bytes_expected) {
            problems.push_back("扫描结束偏移与索引区起点不符（尾部残留 " +
                               std::to_string(trailing) + " B，期望 " +
                               std::to_string(index_bytes_expected) + " B）");
        }
    }
    if (scanned && reader.corrupt_records() > 0) {
        problems.push_back("payload CRC32 校验失败 " + std::to_string(reader.corrupt_records()) +
                           " 条");
    }
    if (scanned && !reader.has_index() && trailing > 0) {
        notes.push_back("文件尾部残留 " + std::to_string(trailing) + " 字节（截断 / 未写完的记录）");
    }

    if (json) {
        PrintJson(path, reader, scanned, check_crc, trailing, problems, notes);
    } else {
        PrintText(path, reader, scanned, check_crc, trailing, problems, notes, dump);
    }
    return problems.empty() ? 0 : 2;
}

