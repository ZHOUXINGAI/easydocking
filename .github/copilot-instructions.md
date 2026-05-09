# EasyDocking Project Instructions

## Efficiency Rules

- **Batch operations**: Read all required files upfront, plan all edits before applying any
- **One-shot builds**: Compile only after ALL file modifications are complete
- **No redundant reads**: Use `tail/grep/awk` via terminal for large log/CSV files instead of `read_file`
- **Aggregate experiments**: Run full batches in a single async command, check results only once at completion
- **Minimize context churn**: Read large files once, avoid re-reading same content
