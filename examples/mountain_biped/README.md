# Mountain Humanoid Biped Example

This example creates a schema `1.5` humanoid biped robot and runs it through a deterministic mountain terrain simulation.

Run locally:

```powershell
py -3.12 examples\mountain_biped_simulation.py --output test_env\mountain_biped\mountain_biped_simulation_report.json --trace-output test_env\mountain_biped\mountain_biped_trace.json
```

Artifacts:

- `test_env/mountain_biped/mountain_biped_simulation_report.json`
- `test_env/mountain_biped/mountain_biped_trace.json`

This is local analytic simulation evidence. It does not replace live Godot physics verification.
