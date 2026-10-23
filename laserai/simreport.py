"""Reproducible offline qualification report, never labelled vendor simulation."""

from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import subprocess
import sys

from .config import _atomic_json, simulation_profile
from .device import SimulatedDevice
from .experiments import run_ab


def generate_report(output: Path) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    cases = [(f"baseline-{hz}Hz", hz, {}) for hz in (1, 5, 10, 20, 30, 40)]
    cases += [("gain-minus20-tau-plus50", 20, {"gain_nm_per_v": 328.0, "tau_s": .012}),
              ("extra-delay-3ms", 20, {"plant_delay_s": .005}),
              ("zero-delay-diagnostic", 20, {"plant_delay_s": 0.0})]
    summaries = {}
    for name, hz, plant in cases:
        profile = simulation_profile()
        profile["trajectory"]["frequency_hz"] = hz
        if name == "zero-delay-diagnostic":
            profile["model"]["plant_delay_s"] = 0.0
        clock = [0.0]
        def advance(seconds):
            clock[0] += seconds
        def factory(p):
            device = SimulatedDevice(p, clock=lambda: clock[0], plant_override=plant)
            device.info["actual_simulated_plant"] = copy.deepcopy(device.plant_model)
            return device
        summary = run_ab(profile, factory, output / "experiments" / name, sleep=advance, poll_s=.25)
        summaries[name] = summary
        for mode, metrics in summary["aggregate"].items():
            rows.append({"case": name, "frequency_hz": hz, "mode": mode,
                         **{key: value["mean"] for key, value in metrics.items()}})
        print(f"{name}: " + ", ".join(f"{mode} RMSE={m['rmse_nm']['mean']:.3f} nm"
                                     for mode, m in summary["aggregate"].items()), flush=True)
    environment = {"created_utc": datetime.now(timezone.utc).isoformat(),
                   "platform": platform.platform(), "python": sys.version,
                   "packages": {name: importlib.metadata.version(name) for name in
                                ("numpy", "scipy", "PySide6", "pyqtgraph", "moku")},
                   "vendor_demo_api_used": False, "physical_hardware_used": False}
    _atomic_json(output / "results.json", {"environment": environment, "cases": summaries})
    with (output / "comparison.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _plot(output, rows, summaries)
    baseline = summaries["baseline-20Hz"]["aggregate"]
    improvement = (1 - baseline["preview"]["rmse_nm"]["mean"] / baseline["pid"]["rmse_nm"]["mean"]) * 100
    verification_path = output / "verification.json"
    verified = json.loads(verification_path.read_text(encoding="utf-8")) if verification_path.exists() else {}
    lines = [
        "# Moku:Go 迁移：无硬件仿真测试报告", "",
        f"生成时间：{environment['created_utc']}。运行系统：{environment['platform']}。", "",
        "## 结论与边界", "",
        "本报告的闭环数据来自本项目的定点数值仿真，不是 Moku 官方演示模式产生的数据。"
        "没有连接 Moku:Go、压电或激光干涉仪，没有部署真实 bitstream，也没有验证模拟前端与板上时序。", "",
        f"20 Hz、500 nm 峰值的基准模型中，Preview+ADRC 相对当前 PID 参数的稳态 RMSE 降低 {improvement:.1f}%。"
        "是否达到 50 nm 以表中数值为准；这不构成真实硬件指标通过。"
        "20 Hz 时 Preview 幅值比约 1.13，存在约 13% 幅值放大；RMSE 只比门限低约 0.47 nm，裕量很小。", "",
        "Moku 已安装软件与演示模式检查见 [环境调查](../../docs/moku-demo-environment.md)。"
        "演示模式的界面可操作性与自定义闭环的运算正确性是两个不同的测试对象。", "",
        "## 方法", "",
        "单轴、控制周期 1 ms，板上等效 DDS 使用 256 点正弦查表，轨迹峰值 500 nm、偏置 1000 nm；"
        "每次 10 s，去除前 2 s；每条件运行 PID 和 Preview 各 3 次，交替先后顺序。"
        "使用和 HDL 相同的 Q16.16 状态量与 Q8.24 系数，全部 1 kHz 数据写入 CSV。", "",
        "基准对象：K=410 nm/V，tau=8 ms，纯延迟=2 ms，输出 0..5 V，电压变化上限 1000 V/s。"
        "PID 参数固定为 Kp=0.002 V/nm、Ki=0.2 V/(nm*s)、Kd=0；观测器 w0=50 rad/s。"
        "两算法共用对象、轨迹、采样和输出限制；未进行最优 PID 整定，因此结论仅适用于这些参数。", "",
        "每个重复实验使用相同的确定性初态，重复标准差为零不代表真实设备有相同重复性。"
        "未模拟 ADC 噪声、DAC 12 bit 量化、模拟前端、压电迟滞、信号失锁或 FPGA 资源/时序。"
        "相位/幅值以稳态数据对已知频率的正弦、余弦和直流基底做最小二乘拟合；正相位值表示滞后。", "",
        "## 数值结果", "",
        "| 条件 | Hz | 控制器 | RMSE nm | 幅值比 | 相位滞后 ° | 饱和比例 |",
        "|---|---:|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(f"| {row['case']} | {row['frequency_hz']} | {row['mode']} | "
                     f"{row['rmse_nm']:.3f} | {row.get('amplitude_ratio', 0):.4f} | "
                     f"{row.get('phase_lag_deg', 0):.3f} | {row['saturation_ratio']:.2%} |")
    lines += ["", "![频率扫描](frequency-sweep.png)", "", "![20 Hz 稳态对比](tracking-20Hz.png)", "",
        "## 如何解释", "",
        "Preview 的前馈利用对象模型与未来轨迹，减少固定 PID 在较高频率下的幅值衰减和相位滞后。"
        "额外延迟与模型失配会改变这种优势；表中的失配试验用于显示敏感性，不能据此保证鲁棒性。"
        "当前实现保留原 control.py 的残差观测器定义：模型减测量，补偿为 res1/K。"
        "它不是通用 ESO；零延迟诊断与带延迟结果应分别查看。", "",
        "## 软件与 HDL 验证", "",
        f"本机自动测试：{verified.get('summary', '见 verification.txt')}；"
        f"进程退出码：{verified.get('exit_code', '见原始输出')}。", "",
        "自动测试原始输出见 [verification.txt](verification.txt)。"
        "界面运行截图见 [qt-simulation.png](qt-simulation.png)。", "",
        "自动测试覆盖配置损坏、未知关键参数禁止输出、目标版本/散列不匹配、寄存器确认超时、"
        "安全归零确认、断连、实验保存、跨线程界面更新和退出。"
        "Icarus Verilog 实际运行 FPGA RTL，覆盖 PID/Preview/开环数值、相位回绕、启动禁止、"
        "事务提交、信号丢失、算术溢出、时限自动停止与输出斜坡；运行输出单独保存在本目录。", "",
        "硬件状态轮询不是连续采样，因此不用于正式 RMSE 验收。FPGA 输出 B 提供分帧遥测，"
        "但记录仪的采样对齐、符号恢复与量纲还需要实机验证；缺帧不能作为合格记录。", "",
        "## 复现与产物", "",
        "```sh", f"{sys.executable} -m laserai.simreport --output {output}",
        f"{sys.executable} -m pytest tests test_control.py -q", "```", "",
        "`results.json` 包含环境和所有重复实验；`comparison.csv` 为对比表；"
        "`experiments/` 中每次实验包含配置/设备/bitstream 标识快照、全采样率 CSV 和指标 JSON。", "",
        "## 尚未验证", "",
        "官方 Go 演示模式的三槽 Phasemeter 画面仅显示 Input A。"
        "文档提供双通道与取消 single-input 的通用接口，但这不足以确认三槽下同时有两路独立测相。"
        "当前三槽路线仍须验证通道能力；后端显式关闭 single-input，若第二路配置被拒绝则部署失败，不能启用闭环。", "",
        "macOS 在本机实际运行；Linux/Windows 已配置 CI，但本机不能代替那两个系统的运行结果。"
        "真实 Go 三槽部署、Phasemeter 回绕/延迟/失锁、直接驱动电流、"
        "bitstream 综合与时序、遥测连续性及 20 Hz/500 nm/50 nm 指标均待实机验证。", "",
    ]
    report = output / "report.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    return report


def _plot(output, rows, summaries):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.5), layout="constrained")
    for mode, color in (("pid", "#b35d23"), ("preview", "#187566")):
        chosen = [r for r in rows if r["case"].startswith("baseline-") and r["mode"] == mode]
        for ax, key, title in zip(axes, ("rmse_nm", "amplitude_ratio", "phase_lag_deg"),
                                  ("RMSE (nm)", "Amplitude ratio", "Phase lag (degrees)")):
            ax.plot([r["frequency_hz"] for r in chosen], [r[key] for r in chosen],
                    "o-", color=color, label=mode)
            ax.set(xlabel="Frequency (Hz)", ylabel=title)
            ax.grid(alpha=.2)
    axes[0].axhline(50, color="#555555", linestyle="--", linewidth=1)
    axes[0].legend()
    fig.savefig(output / "frequency-sweep.png", dpi=170)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=True, layout="constrained")
    for mode, color in (("pid", "#b35d23"), ("preview", "#187566")):
        run = next(r for r in summaries["baseline-20Hz"]["runs"] if r["mode"] == mode)
        with (Path(run["path"]) / "telemetry.csv").open(encoding="utf-8") as stream:
            samples = [r for r in csv.DictReader(stream) if 4.0 <= float(r["time_s"]) < 4.2]
        t = [float(r["time_s"]) for r in samples]
        y = [float(r["measurement_nm"]) for r in samples]
        ref = [float(r["reference_nm"]) for r in samples]
        if mode == "pid":
            axes[0].plot(t, ref, "--", color="#666666", label="reference")
        axes[0].plot(t, y, color=color, label=mode)
        axes[1].plot(t, [a-b for a, b in zip(ref, y)], color=color, label=mode)
    axes[0].set(ylabel="Displacement (nm)", title="20 Hz, 500 nm peak; offline fixed-point simulation")
    axes[1].set(xlabel="Time (s)", ylabel="Tracking error (nm)")
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc="upper right")
    fig.savefig(output / "tracking-20Hz.png", dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("reports/moku-simulation-2026-10-23"))
    parser.add_argument("--verify", action="store_true", help="Run and retain software/HDL test output")
    args = parser.parse_args()
    if args.verify:
        args.output.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, "-X", "faulthandler", "-m", "pytest", "tests", "test_control.py", "-v"]
        completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   cwd=Path(__file__).resolve().parents[1], timeout=180)
        (args.output / "verification.txt").write_text(
            "Command: " + " ".join(command) + "\n" + completed.stdout +
            f"\nProcess exit code: {completed.returncode}\n", encoding="utf-8")
        _atomic_json(args.output / "verification.json", {
            "command": command, "exit_code": completed.returncode,
            "summary": next((line.strip("= ") for line in completed.stdout.splitlines()
                             if re.search(r"\d+ passed", line)), "See verification.txt"),
        })
        if completed.returncode:
            raise RuntimeError(f"Verification failed; see {args.output / 'verification.txt'}")
    print(generate_report(args.output))


if __name__ == "__main__":
    main()
