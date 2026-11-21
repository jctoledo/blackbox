# Code-review skill trial, 2026-11-21

## Result and decision

All three arms found all five planted defects with correct mechanisms and fixes, and no concrete false positives. The stock/adapted skill reports added direct requirement quotes and separate Standards/Spec reporting. This trial provides **no evidence of improved defect recall** over the existing fresh correctness review.

Keep the structured report format and repository overrides. Keep the stock skill available as an optional way to obtain independent axes; do not require extra reviewer agents for every change or claim general superiority. A harder repeated trial would be needed to establish a quality advantage. Research and writing-for-agents were used separately in the discovery work; this trial does not evaluate them, diagnosing-bugs or tdd.

## Method fixed before reports

Baseline source: public commit `2b9ea4b`. An isolated control commit added `REVIEW_EVAL_SPEC.md`; a second local-only commit planted defects. Three detached worktrees had the identical resulting tree (`bec5b6e30aba775f1510fd153b08a38e4305517e`). No planted commit was pushed or merged.

The [scoring key](code-review-2026-11-21-key.md) was written before any arm ran. The [fixture patch](code-review-2026-11-21.patch) reproduces the input from the baseline. Apply it with `git apply --unidiff-zero` only to a disposable checkout of that exact baseline; it deliberately introduces failures.

All reviewers inherited the session's GPT-6.1-Sol model; no model overrides were supplied. They reviewed source only and could inspect tests. They could not edit, execute tests/builds, access sibling arms/key, or mutate the tracker. Root validated failure signals separately on lat. Arms ran sequentially to respect the four-agent limit; the stock/adapted arms each spawned two fresh parallel axis reviewers. This compares workflows, not equal compute budgets.

| Arm | Procedure | Reviewer/coordinator agents | Defects | Concrete false positives | Score / 18 |
| --- | --- | --- | --- | --- | --- |
| A | Existing independent correctness review using repository guidance | 1 | 5/5 | 0 | 17 |
| B | Pinned stock code-review skill | 3 | 5/5 | 0 | 18 |
| C | Same skill with repository authority, supplied scope and same-cause sweep made explicit | 3 | 5/5 | 0 | 18 |

The one-point difference was direct quotation of requirements: A referenced requirement numbers accurately; B/C quoted them. All arms earned the written-standard citation point and identified both premature-clear paths. Heuristic smells in B/C were labelled judgments, not counted as bugs or false positives. No arm claimed to have run tests.

## Known causes and independent evidence

| Cause | What all arms identified | Lat evidence |
| --- | --- | --- |
| Mutable ownership after await | History IDs reread globals while best/count update captured owner | Switch, deactivate and delayed-open regressions fail on the planted code |
| Premature durable-success signal | Request success and cursor exhaustion precede transaction commit | Abort-after-request and per-track rollback regressions fail |
| Wrong metric conversion | 50–100 km/h thresholds use mph conversion | Combined CSV interval regression fails |
| Swapped compass/ENU axes | Timer receives cosine/sine in reversed components | Added diagonal southeast crossing reproducer fails on plant |
| External firmware output | ESP-IDF cannot discover project Git metadata above OUT_DIR | Prior real external-target image reports version `1`; exact-tag release guard rejects it |

The browser fixture run had six failing checks. The four existing queue tests still passed the planted axis swap at their cardinal-heading boundary; a southeast-heading oracle then failed. That revealed a real coverage gap, and the corresponding diagonal forward/reverse regression belongs in production tests. Root restored the ordinary lat review checkout afterwards.

## Limits

One small, explicitly specified fixture creates a ceiling effect: all arms found everything. The rubric, prompts, current model and visible repository guidance influence results. Extra agents cost resources; no latency/token-cost comparison was measured. The score difference is reporting traceability, not a demonstrated mathematical or firmware-diagnosis advantage. There were no blind real-world driving failures, runtime reviewer runs or statistically powered repeats.

The repository workflow therefore protects known rules, uses fresh review proportional to risk, and treats benchmark results as local evidence. Installed skills are tools, not certification that Blackbox or an agent is reliable.
