# Use the Workshop Environment

This document provides practical steps for common tasks in the `manif-dev` workshop environment.

## How to start a workshop session

To launch a new instance and access it via a terminal:

```bash
workshop launch
workshop shell
```

### Available Actions

You can execute these commands via `workshop run <action> [args]`:

| Action | Description |
| --- | --- |
| `ssh-import-id-gh` | Imports SSH keys from GitHub |
| `opencode` | Runs Opencode |
| `just` | Runs `just` command runner |
| `build` | Runs `just build build_dir="${HOME}/build"` |
| `clean` | Runs `just clean build_dir="${HOME}/build"` |
| `test` | Runs `just test build_dir="${HOME}/build"` |
| `docs-make` | Generates HTML documentation |
| `docs-serve` | Serves documentation locally |
| `docs-clean` | Cleans documentation build files |

---

## How to configure Opencode

To use your local Opencode configuration and credentials within the workshop:

Stop the current instance:

```bash
workshop stop robotics-snapcrafter
```

Remount your configuration and data:

```bash
workshop remount robotics-snapcrafter/opencode:opencode-config ~/.config/opencode
workshop remount robotics-snapcrafter/opencode:opencode-data ~/.local/share/opencode
```

Restart the workshop:

```bash
workshop start robotics-snapcrafter
```

Later on, to continue your last Opencode session:

```bash
workshop run opencode --continue
```

---

## How to connect VS Code

To use VS Code for the workshop environment:

Import your GitHub SSH key:

```bash
workshop run ssh-import-id-gh <GH_HANDLE>
```

Connect via Remote-SSH:

```bash
code --folder-uri vscode-remote://ssh-remote+workshop@$(workshop info | awk '/hostname/{print $2}')/project
```

Recommended extensions:

- [davidanson.vscode-markdownlint](https://github.com/DavidAnson/vscode-markdownlint)
- [ms-vscode.cmake-tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
- [sst-dev.opencode](https://marketplace.visualstudio.com/items?itemName=sst-dev.opencode)
- [FedaykinDev.openchamber](https://marketplace.visualstudio.com/items?itemName=FedaykinDev.openchamber)

> [!IMPORTANT]
> If VS Code refuses the SSH connection after a workshop refresh, clear the stale host key:
> `ssh-keygen -R "$(workshop info | awk '/hostname/{print $2}')"`

---

## How to connect Inference Snaps

To enable access to inference models via an Inference Snap, connect the required interfaces:

```bash
workshop connect robotics-docs-dev/inference-snaps:inference-snap robotics-docs-dev/system:inference-snap
```
