# Approved disposable exploration removal

User confirmed these two files disposable on 2026-10-05. No other exploration
artifacts or processes are authorized for removal by this confirmation.
Before removal, both were regular single-link files; fuser found no open users.

| Path | Logical bytes | Allocated bytes | SHA256 |
| --- | ---: | ---: | --- |
| .ros_logs/aws_warehouse_runner_01/evidence/exploration/exploration_0.db3 | 445693952 | 445698048 | fc734ce95f0260b2e92d80f738a5b85c1b42654b814781a91b842af044da403f |
| .ros_logs/aws_warehouse_runner_01/evidence/exploration_diagnostics.jsonl | 156871222 | 156876800 | 93bf5f112b181e835b6eb21e47942c620c4185c09a528f400bbe84f337f9296a |

Before category usage: evidence1270641064 logical/1288187904 allocated;
logs359525995 logical/416591872 allocated. Reclamation target for evidence is
500000000 in both measures. Exact two-path removal command and resulting usage
will be retained below after execution.

Executed `rm --` with exactly the two absolute paths above, exit0. Canonical
storage helper afterward exited0: evidence668075890 logical/685613056 allocated;
logs359525995 logical/416591872 allocated. Both approved files are removed.
The triggered evidence reclamation target is still pending; ordinary admission
success does not replace the required <=500000000 reclamation verification.
