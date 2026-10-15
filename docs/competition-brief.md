# Competition brief

Reviewed October 8, 2026. Source: [official competition overview](https://msoe-maic.com/library/?nav=Competitions&article=Rockwell-NVIDIA-Fall-2026) and its linked dataset and resource guides. Recheck published requirements before submission; detailed judging weights and award categories are pending.

## Deadlines and deliverables

| Milestone | Date and time, Central Time | Deliverable |
| --- | --- | --- |
| Kickoff | October 8, 4–6 PM | Rockwell Automation HQ, Milwaukee |
| Team plan | October 14, 11:59 PM | Short problem statement and resource/support requests |
| Final submission | November 1, 11:59 PM | Approximately five-minute video, one-page summary, team repository, references and credits |
| Finals | November 5, 5–8 PM | MSOE Diercks Hall presentation |

The overview says judges expect a solution operating on the real CSI production line or FANUC arm, a specific problem, a business case relevant to other manufacturers, measured results against a baseline, and an honest account of failures. A simulation-only demo would not satisfy that stated hardware expectation.

## Track comparison

| Track | Provided opportunity | Important limitation | Team V assessment, not an organizer ranking |
| --- | --- | --- | --- |
| Final vial inspection | Overhead inspection of two 30-position fixtures; missing/crooked caps, damaged vials, occupancy | Transparent bodies, glare, cap defects, and fill-progress context; missed defects matter more than false alarms | Strong fallback: relatively contained integration and clear metrics |
| Fluid mixing | Synchronized tank, filling, and cap/seal cameras; recipe sequence and carryover | Only 30 labeled vials across 10 recipes; twin lacks tanks and approximates liquid color | Preferred investigation: connect detection to prevention and measurable waste reduction |
| Grasping and sorting | FANUC ER-4iA, SCHUNK gripper, three cameras, twin, printable fingers | Twin/calibration access details pending when reviewed; physical integration and supervised access are dependencies | Strong candidate if the team has robotics/controls experience and early hardware access |

## Data and physical access

The dataset guides direct teams to generate training data using digital twins and avoid training on supplied real-line examples. Real data can inform understanding of the station and should provide evaluation evidence. Keep a final real test split untouched by training, prompt examples, threshold selection, and augmentation decisions. Ask organizers to clarify the boundary for calibration and development evaluation; the collection guide also discusses combined-data comparisons, which should not be treated as overriding the track-specific training direction.

The fluid twin supplies fill and cap/seal stations and matched cameras, but no tanks; liquid color is approximate. Do not assume it simulates fluid dynamics or accurate spectral mixing. Dataset 3 is a separate robot setup, not part of the vial line.

Visits, extra data, and supervised hardware tests must be requested early through #data-and-testbed-requests, tagging Shamar Webster. Additional collection is not guaranteed. Teams may not independently alter cameras, PLCs, robot controllers, FactoryTalk, recipes, production programs, or safety systems, or attach network devices or gripper components. The plant manager must approve equipment interaction.

For mixing, the guide states: total vial volume at most 80 mL; a selected component should be at least 10 mL in practice; T-200 at most 50 mL; do not use T-300; do not change recipes without plant-manager guidance. Dispensing uses T-200, then T-400, then T-100, skipping unused tanks. These constraints must appear in any proposed recipe or sequence experiment.

## Source links

- [Vial inspection](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Vial-Inspection)
- [Fluid mixing](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Fluid-Mixing)
- [Grasping and sorting](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting)
- [Data collection and hardware testing](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Data-Collection)
- [Compute](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute)
- [Coding agents](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Coding-Agents)
- [Tools and resources](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Resources)
- [Shared data folder](https://drive.google.com/drive/folders/1GWuHV2WUNLztq_Fga6xBH7qFzhxDaac8?usp=sharing)

The linked ALL AI Network workspace manages team repositories and submissions. Confirm that the user-supplied GitHub repository is accepted for submission or linked to the organizer-managed team repository before relying on it as the final submission location.
