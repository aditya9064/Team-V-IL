# Real-world factory limitations (research, Oct 17, 2026)

Why it matters: judges want a specific factory problem and a business case. These are the limitations
we found for robotic pick-and-sort, and how our cell could address each one.

| # | Limitation | Evidence | Can our cell show it? |
| --- | --- | --- | --- |
| 1 | **Small error rates compound.** 99.5% vision × 99.5% grip ≈ 99% per part → ~36% chance to empty a 100-part bin with no failure. Robots retry the *same* failed grasp, cutting picks per hour. | [Control Engineering](https://www.controleng.com/articles/what-happens-when-automation-systems-fail), [UC Berkeley CASE 2020](https://arxiv.org/abs/2007.10420), [VentureBeat on ABB's metric](https://venturebeat.com/business/as-robotic-grasping-improves-rivals-debate-the-metrics-of-success) | Yes — self-check every pick (wrist camera grip check, top camera bin check), different-grasp retry |
| 2 | **Downtime is expensive.** Automotive ≈ $2.3M per hour, 2× 2019. | [Siemens True Cost of Downtime 2024](https://assets.new.siemens.com/siemens/assets/api/uuid:1b43afb5-2d07-47f7-9eb7-893fe7d0bc59/TCOD-2024_original.pdf) | Yes — recover without stopping |
| 3 | **Reprogramming per product change.** High-mix lines change over several times a shift; integration costs ~4–5× the robot. | [Control Engineering](https://www.controleng.com/automation-solutions-for-high-mix-manufacturing/), [Assembly Magazine](https://assemblymag.com/articles/83900-harnessing-high-mix-low-volume), [MIT Sloan](https://mitsloan.mit.edu/node/29144) | Yes — re-task with the ManipBench instruction text |
| 4 | **Skills shortage.** Up to 3.8M jobs needed 2024–2033, 1.9M may go unfilled; small manufacturers cite cost and lack of robotics experience. | [Deloitte / Manufacturing Institute](https://www.deloitte.com/us/en/Industries/industrial-construction/articles/future-of-manufacturing-skills-gap-study.html), [Automate.org](https://www.automate.org/robotics/industry-insights/small-manufacturers-are-supposed-to-be-automations-next-big-market-what-s-stopping-them/boa) | Yes — no programmer needed for a new sort rule |
| 5 | **Bumped cameras / calibration drift.** Plants spend weeks blaming lighting before checking calibration; recalibrating without stopping is an open problem. | [iFactory](https://ifactoryapp.com/industries/automotive-manufacturing/robot-vision-calibration-hand-eye-automotive-accuracy), [arXiv 2004.12611](https://arxiv.org/pdf/2004.12611) | Sim only — we may not move real cameras |
| 6 | **Lighting changes, shiny/clear parts.** Skylights confuse vision; metal glare; transparent parts blend in. | [Control.com](https://control.com/technical-articles/uncontrolled-machine-vision-lighting-for-infrared-and-robots/), [Mech-Mind](https://www.mech-mind.com/blog/why-transparent-objects-are-so-hard-for-robots.html) (vendor) | Mostly sim (Cosmos Transfer robustness) |
| 7 | **Clutter, piles, bin corners.** Commercial systems often avoid it by restricting items. | [MetaGraspNet](https://arxiv.org/pdf/2208.03963), [Automate.org](https://www.automate.org/robotics/industry-insights/robotic-bin-picking-the-holy-grail-in-sight) | Partly — matches our 30% "uncertain" refusals; risky under the 8 mm / 2 mm limits |

**Recommendation:** combine 1–4 into one pitch — *"a sorting robot you re-task with one sentence, that
checks its own work and recovers without stopping the line"* — and show 5 as a sim-only bonus.
Vendor sources (Mech-Mind, OnRobot, KEYENCE) are marked as such and weaker evidence.
