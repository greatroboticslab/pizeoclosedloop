"""Host ABI, independent numeric oracle and actual HDL simulation checks."""
import math
from pathlib import Path
import shutil
import subprocess

import pytest

from laserai.protocol import (
    ABI_MAGIC, CONFIG_WORDS, Control, FixedController, board_reference,
    compile_config, decode_status, encode_fixed, prepare_coefficients,
    staging_writes, decode_telemetry_frames, board_disturbance,
)


def parameters():
    return dict(
        duration_ticks=45, heartbeat_timeout_ticks=0, phase_period_counts=65536,
        nm_per_phase_count=1, direction=1, v_min=0, v_max=5, v_safe=0,
        slew_per_tick=1, max_abs_error=10000, signal_threshold=100,
        signal_timeout_ticks=3, dac_counts_per_volt=6553.4,
        waveform=1, amplitude=100, offset=800,
        phase_step=round(20 / 1000 * 2**32), phase_initial=0,
        duty=.4, symmetry=.3, edge_fraction=.05, disturbance_mode=0,
        disturbance_amplitude=0, disturbance_phase_step=round(70 / 1000 * 2**32),
        disturbance_phase_initial=round(.3*2**32), disturbance_start_ticks=3,
        disturbance_duration_ticks=11, disturbance_seed=0xC0FFEE42,
        trajectory_seed=0xDEADBEEF,
        **prepare_coefficients(gain_nm_per_v=410,tau_s=.08,dt_s=.001,
                               plant_delay_s=.01,sensor_delay_s=.003,
                               kp=.002,ki=.01,kd=.00001,d_filter_n=20,w0=50,v_max=5),
    )


def test_complete_config_and_transaction():
    words=compile_config(parameters())
    assert len(words)==CONFIG_WORDS
    writes=staging_writes(words,commit_id=7)
    assert writes[0]==(Control.MAGIC,ABI_MAGIC)
    assert writes[-1]==(Control.COMMIT_ID,7)
    assert writes[3]==(Control.WRITE_SEQUENCE,1)
    status=decode_status([ABI_MAGIC,7,1,0,10,0,encode_fixed(-2),encode_fixed(50),encode_fixed(1.25),0,36,2])
    assert status.measurement_nm == -2 and status.voltage_v == 1.25


@pytest.mark.parametrize("field,value",[("v_safe",None),("v_max",6),("direction",0),
    ("nm_per_phase_count",1e-12),("model_b",200),("sensor_delay_ticks",256),
    ("offset",32760),("slew_per_tick",float("nan")),("tick_divider",1.5),
    ("waveform",6),("duty",1.01),("symmetry",0),("symmetry",1),
    ("edge_fraction",.201),("disturbance_amplitude",5.01),
    ("disturbance_mode",5),("disturbance_start_ticks",-1),
    ("disturbance_seed",2**32),("trajectory_seed",-1)])
def test_invalid_config_is_rejected(field,value):
    values=parameters(); values[field]=value
    with pytest.raises(ValueError): compile_config(values)


def test_coefficients_match_independent_legacy_matrix_exponential():
    np=pytest.importorskip("numpy")
    from control import PlantModel, PreviewADRC
    p=parameters()
    oracle=PreviewADRC(PlantModel(),w0=50,dt=.001)
    assert np.allclose([[p["observer_a11"],p["observer_a12"]],
                        [p["observer_a21"],p["observer_a22"]]],oracle.Ad_res,atol=1e-12)
    assert np.allclose([p["observer_b1"],p["observer_b2"]],oracle.Bd_res,atol=1e-12)


@pytest.mark.parametrize("algorithm",["pid","preview"])
def test_fixed_controller_tracks_existing_numeric_laws(algorithm):
    from control import PlantModel, PreviewADRC, SoftwarePID
    values=parameters(); values["slew_per_tick"]=5
    words=compile_config(values)
    fixed=FixedController(words)
    model=PlantModel(theta_sensor_s=.003)
    oracle=(SoftwarePID(model,.002,.01,.00001,20) if algorithm=="pid" else
            PreviewADRC(model,w0=50,dt=.001))
    for k in range(200):
        r,preview=board_reference(words,k)
        measurement=600+100*math.sin(k*.11)
        actual=fixed.update(algorithm,r,measurement,preview)
        expected=(oracle.update(r,measurement,.001) if algorithm=="pid" else
                  oracle.update(*preview,measurement))
        assert actual==pytest.approx(expected,abs=.002)


@pytest.mark.parametrize("algorithm,command",[("pid",1),("preview",2),("open_loop",4)])
@pytest.mark.parametrize("waveform",range(6))
@pytest.mark.parametrize("disturbance_mode",range(5))
def test_hdl_numeric_vectors_and_safety(tmp_path,algorithm,command,waveform,disturbance_mode):
    compiler=shutil.which("iverilog")
    runtime=shutil.which("vvp")
    if not compiler or not runtime:
        pytest.skip("Icarus Verilog is required for actual HDL simulation")
    root=Path(__file__).resolve().parents[1]
    values=parameters()
    values.update(waveform=waveform,phase_initial=round(.83*2**32),
                  disturbance_mode=disturbance_mode,disturbance_amplitude=-.35)
    words=compile_config(values)
    # Faster simulation divider changes only clock timing, not numeric coefficients.
    words[0]=100
    config=tmp_path/"config.hex"
    config.write_text("\n".join(f"{x&0xffffffff:08x}" for x in words),encoding="ascii")
    fixed=FixedController(words)
    vectors=[]
    for k in range(20):
        reference,preview=board_reference(words,k)
        measured=k%7
        disturbance=board_disturbance(words,k)
        v=fixed.update(algorithm,reference,measured,preview,disturbance_v=disturbance)
        vectors.append(f"{measured&0xffffffff:08x}{encode_fixed(v)&0xffffffff:08x}"
                       f"{encode_fixed(reference)&0xffffffff:08x}{encode_fixed(disturbance)&0xffffffff:08x}")
    vector_file=tmp_path/"vectors.hex"
    vector_file.write_text("\n".join(vectors),encoding="ascii")
    binary=tmp_path/"sim"
    subprocess.run([compiler,"-g2012","-I",str(root/"fpga"),"-s","testbench","-o",str(binary),
                    str(root/"fpga/laser_control.sv"),str(root/"fpga/testbench.sv")],check=True,capture_output=True,text=True)
    result=subprocess.run([runtime,str(binary),f"+config={config}",f"+vectors={vector_file}",
                           f"+algorithm={command}","+count=20"],capture_output=True,text=True,timeout=30)
    assert result.returncode==0, result.stdout+result.stderr
    assert "PASS:" in result.stdout
    frames=[]
    for line in result.stdout.splitlines():
        if line.startswith("FRAME "):
            frames.extend(int(symbol,16) for symbol in line.split()[1:])
    decoded=decode_telemetry_frames(frames)
    assert len(decoded)==20
    for k,frame in enumerate(decoded):
        assert frame.measurement_nm==k%7
        assert encode_fixed(frame.voltage_v)==int(vectors[k][8:16],16)
        assert frame.reference_nm==board_reference(words,k)[0]
        if k: assert frame.tick-decoded[k-1].tick==1


def test_telemetry_resynchronizes_after_corruption():
    frame=[0xA55A,1,0,42,0,0,0,1,0,2,0,0,0,7,1]
    checksum=0
    for value in frame: checksum^=value
    frame.append(checksum)
    damaged=frame.copy(); damaged[8]^=1
    decoded=decode_telemetry_frames([123,*damaged,456,*frame])
    assert len(decoded)==1 and decoded[0].tick==42 and decoded[0].commit_id==7


def test_abi2_rejects_old_status_and_incomplete_old_config():
    with pytest.raises(ValueError): decode_status([0x4C415301]+[0]*15)
    p=parameters(); p.pop("trajectory_seed")
    with pytest.raises(ValueError,match="trajectory_seed"): compile_config(p)


def test_pulse_square_ramp_endpoints_and_smoothed_edges():
    p=parameters(); p.update(phase_step=2**30,phase_initial=0,edge_fraction=0,duty=.5,symmetry=.25)
    p["waveform"]=2; words=compile_config(p)
    assert [board_reference(words,k)[0] for k in range(4)]==[900,900,700,700]
    p["waveform"]=4; words=compile_config(p)
    assert [board_reference(words,k)[0] for k in range(4)]==[900,900,800,800]
    p["waveform"]=3; words=compile_config(p)
    assert [board_reference(words,k)[0] for k in range(4)]==pytest.approx([700,900,833.33333,766.66666],abs=2e-5)
    p.update(waveform=2,edge_fraction=.1); words=compile_config(p)
    assert board_reference(words,0)[0]==800
    assert board_reference(words,1)[0]==900
    assert board_reference(words,2)[0]==800


def test_seeded_noise_preview_and_disturbance_window():
    p=parameters(); p.update(waveform=5,disturbance_mode=4,disturbance_amplitude=.5)
    words=compile_config(p)
    assert board_reference(words,8)[1][0]==board_reference(words,8+words[19])[0]
    assert board_reference(words,8)==board_reference(words,8)
    different=words.copy(); different[46]^=1
    assert board_reference(words,8)!=board_reference(different,8)
    assert board_disturbance(words,2)==0
    assert board_disturbance(words,14)==0
    assert any(board_disturbance(words,k)!=0 for k in range(3,14))


def test_disturbance_shares_single_output_limit_and_slew():
    p=parameters(); p.update(slew_per_tick=.1)
    words=compile_config(p)
    fixed=FixedController(words)
    assert fixed.update("open_loop",0,0,disturbance_v=4)==encode_fixed(.1)/65536
    assert fixed.update("open_loop",0,0,disturbance_v=-4)==0
