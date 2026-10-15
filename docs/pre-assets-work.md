> Historical planning/research document. For current scope, resource status, verified results and next steps, read [START-HERE.md](START-HERE.md). Statements below describe an earlier project stage.

# Work before the FANUC assets arrive

The team plan draft is ready for review in team-plan-draft.md. No robot model, hardware interface, training result, or physical success is assumed.

## Start now

1. Confirm captain/member names and assign the four ownership pairs in the strategy.
2. Agree on a small evaluation contract: an attempt is one pick; a task can include a bounded retry; a successful task requires the intended destination and no unresolved failure. Decide the retry budget with the mentor before physical use.
3. Use data/trial-log-template.csv to log every attempt. Keep final trials separate from development. Record code/model versions and condition IDs.
4. Build replay-mode perception and outcome-verification components with clearly labeled sample inputs. Coordinate transforms and hardware adapters remain pending calibration/interface files.
5. Review the existing vial imagery and identify candidate synthetic inspection experiments. Do not train on supplied real images or assume their viewpoints match the FANUC cell.
6. Prepare the demo narrative: problem, baseline failure, proposed loop, matched evaluation, limitations, and manufacturer value. Add results only after measurement.

## Await assets or access

Robot scene loading, grasp physics, real coordinate transforms, live robot/gripper commands, and validated camera localization require the supplied robot package and mentor guidance. Physical tests require a confirmed time and supervisor. The dashboard plan form requires personal sign-in before its fields can be inspected and filled.
