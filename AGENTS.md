# Agent notes

## Tests

While iterating, run the quick set:

```
python -m pytest -m "not slow" -n auto
```

Before pushing a pull request, run the full set. CI runs this same command:

```
python -m pytest -n auto
```

`slow` marks the closed-loop laps. `-n auto` needs `pytest-xdist`. `pytest.ini` sets `--dist loadgroup`, so the database tests share one worker.
