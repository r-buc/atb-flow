# Agents Instructions

## Codebase Awareness

- Always explore the full codebase before making changes. Understand all scripts, modules, and their interactions to avoid missing dependencies or requirements.
- Read existing code for patterns and conventions. When implementing new logic, follow the same patterns already used in the codebase (e.g., WiFi/Ethernet fallback patterns, error handling, logging).

## Tasmota Source of Truth

- The Tasmota source code at [r-buc/Tasmota](https://github.com/r-buc/Tasmota) is the authoritative reference for all Tasmota APIs and internals.
- Always verify Tasmota Berry API behavior (e.g., `tasmota.wifi()`, `tasmota.eth()`, `tasmota.rtc()`) against the actual Tasmota source code rather than relying on assumptions or external documentation alone.

## Tasmota Berry & Environment Quirk Awareness

- **Native Modules vs. Mocks**: Tasmota on ESP32 *does* natively include several modules like `re` (Regex). Files in the `tests/` directory (like `tests/re.be`) are local mocks intended for standalone PC testing. Do not assume a module is missing in Tasmota just because a mock exists in `tests/`.
- **Identifying Native Modules**: To definitively check if a Berry module exists in Tasmota firmware, do not rely on guesses. Search the Tasmota source code (e.g., `lib/libesp32/berry/default/be_modtab.c` or similar) for `be_native_module(<module_name>)` or check for definitions in the Tasmota `lib/libesp32/berry/` directory.
- **Regex & Memory Constraints**: Be extremely cautious with regex on resource-constrained MCUs. Always use non-greedy matches (e.g., `.*?`) instead of greedy matches (e.g., `.*`) to prevent massive backtracking. Greedy unbounded regexes over payloads will rapidly exhaust MCU memory, triggering GC panics (`be_gc_collect`) and crashing the device.

## CI / Test Validation

- Unless the user explicitly says otherwise, create a pull request when the task is complete.
- Pull request CI workflows are triggered automatically. Always wait for all test runs to complete before considering work finished.
- If any CI job fails, investigate the failure logs, fix the issue, and iterate until all jobs pass.
- Do not finalize work until all CI checks are green.

## Continuous Improvement

- Always update this `AGENTS.md` file when user feedback contains instructions or lessons that apply beyond the scope of a single PR.
- Summarize gathered knowledge in this `AGENTS.md` for future easier retrieval.
- Distill feedback into clear, generic guidelines and add them to the appropriate section (or create a new section if needed).
