---
name: Bug report
about: Report a bug in carla_apollo_bridge
title: ''
labels: kind/bug
assignees: ''

---

## Describe the bug
<!-- A clear and concise description of what the bug is -->

### In what area(s)?
<!-- Remove the '> ' to select -->

> /area runtime

> /area operator

> /area placement

> /area docs

> /area test-and-release


## Steps to Reproduce the Problem

<!-- How can a maintainer reproduce this issue (be detailed) -->

Steps to reproduce the behavior:
1. Run this command in the container to start Dreamview: `./scripts/bootstrap.sh` 
2. Start the bridge and spawn the ego vehicle: `python carla_cyber_bridge/bridge.py` and `python carla_spawn_objects/carla_spawn_objects.py`
3. Start a co-simulation: `Open apollo client: http://localhost:8888`
4. Other operate...


## Expected Behavior

<!-- Briefly describe what you expect to happen -->


## Actual Behavior

<!-- Briefly describe what is actually happening -->


## Screenshots or Video

<!-- If applicable, add screenshots to help explain your problem  -->


## Environments (please complete the following information):
 - System info: [use `uname --all` on LInux]
 - Cuda version [use `nvidia-smi`]
 - Docker version [use `docker -v`]
 - Docker-compose version [use `docker-compose -v`]
 - Versions of other dependent software...


## Additional context

<!-- Add any other context about the problem here -->


## What version of carla_apollo_bridge?

<!-- Delete all but your choice -->

> 0.2.x
> 0.1.x
> edge: output of `git describe --dirty`


## Release Note

<!-- How should the fix for this issue be communicated in our release notes? It can be populated later. -->
<!-- Keep it as a single line. Examples: -->

<!-- RELEASE NOTE: **ADD** New feature in carla_apollo_bridge. -->
<!-- RELEASE NOTE: **FIX** Bug in runtime. -->
<!-- RELEASE NOTE: **UPDATE** Runtime dependency. -->

RELEASE NOTE:
