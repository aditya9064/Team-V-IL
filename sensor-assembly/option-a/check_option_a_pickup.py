"""CLI for independent CPU-only native Option A evidence review."""
import sys
sys.dont_write_bytecode = True
from option_a_pickup_trace_checker import main

if __name__ == '__main__':
    raise SystemExit(main())
