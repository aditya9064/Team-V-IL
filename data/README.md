# Supplied competition data

Source: [Innovation Lab shared folder](https://drive.google.com/drive/folders/1GWuHV2WUNLztq_Fga6xBH7qFzhxDaac8).

Downloaded October 8, 2026. Local files are deliberately excluded from Git; authorized teammates should obtain the originals from the shared folder.

| Local path | Source | Compressed size |
| --- | --- | --- |
| `external/csi_digital_twin.zip` | `Copy of csi_digital_twin.zip` | 212,620,610 bytes |
| `external/csi-vial-fill-dataset.zip` | `csi-vial-fill-dataset.zip` | 709,560,806 bytes |

Extracted directories:

- `external/csi_digital_twin_v1/`: scene layers, geometry, two camera JSONs, previews, and rendering tools.
- `external/csi-vial-fill-dataset/`: three cameras, image indexes, videos, and vial/frame recipe labels.

The dataset archive contains the labels and videos even though some expanded Drive folders returned empty listings during inspection. The connector's 256 MiB download limit prevented fetching the dataset archive through it; the official Google Drive large-file download completed instead. Both ZIPs were extracted with archive-path checks. No supplied executable scripts were run.

To repeat the read-only structural audit from the repository root with Python 3:

```bash
python3 scripts/audit_dataset.py data/external/csi-vial-fill-dataset --out docs/dataset-audit.json
```

Keep supplied raw files intact. Put derived outputs in `processed/` and synthetic examples in `generated/`. Follow the competition's synthetic-training direction and preserve final real evaluation runs. See [dataset findings](../docs/dataset-findings.md).
