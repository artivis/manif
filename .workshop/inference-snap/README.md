# inference-snap SDK

A workshop SDK that configures [opencode](https://opencode.ai) to use locally running inference models served by an [inference snap](https://documentation.ubuntu.com/inference-snaps).

## Overview

This SDK registers the `inference-snaps` provider with opencode, pointing it at the local inference snap endpoint (`localhost:8336`). It connects via the `tunnel` interface so the snap can reach the inference service.

## Configuration

The provider configuration is defined in [`config.json`](config.json), which is a valid standalone opencode config. During setup it is installed to `/etc/opencode/inference-snaps/config.json`.

The `setup-base` hook writes the provider into `/etc/opencode/opencode.json`:

- If no managed config exists, `config.json` is installed directly.
- If a managed config already exists, the `inference-snaps` provider entry is merged in using `jq`, preserving existing settings.
