# Repository maintenance

Preserve upstream localization structure and COPYING. Current public config is config/emcl2.param.yaml. Preserve scan age/duplicate rejection and independent driving watchdog boundaries. Do not merge the divergent GNSS experiment indiscriminately.

## Changes and evidence

Update the authoritative feature/interface/launch documentation in the same change. Link to the owner rather than copying current instructions; preserve Logbooks and historical evaluation conditions. Record repository SHA, dirty diff, environment, input identity, command, counts, exit status, failures/skips and unverified conditions. Distinguish offline checks, simulation and hardware evidence.

Start with related offline tests after reading their fixtures. Use ROS 2 Humble consistently and explicitly select actual source paths and fresh build/install outputs. Keep parallel workers at most two. Never start hardware nodes, enable motion, send robot commands, replay bags or launch simulation without explicit authorization. Preserve safety latches and require manual permission/rearm.

Inspect metadata, filenames and summaries before any large recording/model/log. Narrow searches to relevant source and exclude generated output/data/cache. Store full check logs in generated log/ paths, report concise failures and preserve the original exit code. Do not install dependencies, update upstream, push, rewrite history or change main/master as part of this implementation.
