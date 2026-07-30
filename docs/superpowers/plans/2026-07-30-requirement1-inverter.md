# Requirement 1 Inverter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure the existing three-phase SVPWM inverter for a 32Vrms, 60Hz line output and make its Trip release safe with the 40kHz control ISR.

**Architecture:** Retain the existing phase-voltage feedback loop, bus feedforward, SVPWM generator, and 20kHz inverter carrier. Change only the rated phase reference, nominal modulation denominator, reference clamp, and the inverter startup wait state; the rectifier and ADC timing remain unchanged.

**Tech Stack:** TMS320F280049C C2000 C, ePWM, ADC-triggered ISR, PowerShell policy tests, CCS 12 build tools.

---

### Task 1: Lock down the requirement-1 policy

**Files:**
- Create: `tests/controller_requirement1_inverter_policy.ps1`
- Test: `tests/controller_requirement1_inverter_policy.ps1`

- [ ] Write assertions for 18.4752Vrms phase reference, 2.1044 initial `M`, 60Hz default output, unchanged 20kHz inverter PWM, and two-ISR Trip release delay.
- [ ] Run `powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests/controller_requirement1_inverter_policy.ps1`.
- [ ] Confirm it fails because the current source still contains the 16Vrms line-output reference and lacks the release counter.

### Task 2: Implement the rated output and safe release

**Files:**
- Modify: `controller.c`
- Test: `tests/controller_requirement1_inverter_policy.ps1`

- [ ] Change `INVERTER_MODULATION_DIVISOR_INITIAL` to `2.1044f`.
- [ ] Add `OUTPUT_PHASE_REF_MAX_RMS 20.0f` and clamp `U_OUT_REF` with that macro.
- [ ] Change `U_OUT_REF` to `18.4752f`.
- [ ] Add a two-ISR saturating startup counter and clear ePWM1～3 OST only after the counter reaches two.
- [ ] Reset the counter whenever inverter startup is initialized, cancelled, or completed.
- [ ] Run the requirement-1 policy test and confirm it passes.

### Task 3: Regression and build verification

**Files:**
- Verify: `controller.c`
- Verify: `tests/controller_svpwm_policy.ps1`
- Verify: `tests/control_algorithms_test.exe`

- [ ] Run the new requirement-1 policy and the existing SVPWM policy.
- [ ] Run `tests/control_algorithms_test.exe`.
- [ ] Run `D:\CCS\ccs\utils\bin\gmake.exe -j4 all` from `Debug`.
- [ ] Confirm compilation and linking complete without errors.
