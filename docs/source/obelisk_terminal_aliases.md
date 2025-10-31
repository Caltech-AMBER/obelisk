# Obelisk Terminal Aliases

## `obk-launch`
Launches the `obelisk_bringup.launch.py` with specified arguments.

Usage:
```
obk-launch config=<path> device=<name> auto_start=<True|False>
```
Example:
```
obk-launch config=example.yaml device=onboard auto_start=True
```

## `obk-build`
Builds Obelisk.
Usage:
```
obk-build
```

## `obk-clean`
Cleans the Obelisk build folders. If the build is failing unexpectedly, it can be useful to run this command and try again.
```
obk-clean
```

## State Transitions
### `obk-configure`
Configure all Obelisk nodes.
```
obk-configure <config_name>
```

### `obk-activate`
Activate all Obelisk nodes.
```
obk-activate <config_name>
```

### `obk-deactivate`
Deactivate all Obelisk nodes.
```
obk-deactivate <config_name>
```

### `obk-cleanup`
Cleanup all Obelisk nodes.
```
obk-cleanup <config_name>
```

### `obk-shutdown`
Shutdown all Obelisk nodes.
```
obk-shutdown <config_name>
```

## Convenience Commands

### `obk-start`
Alias for obk-configure.
```
obk-start <config_name>
```

### `obk-stop`
Alias for obk-deactivate and obk-cleanup.
```
obk-stop <config_name>
```

### `obk-kill`
Alias for obk-shutdown.
```
obk-kill <config_name>
```

### `obk-help`
Display the help message.

In all the above commands, `<config_name>` refers to the config field of the config file used for launching obelisk.
