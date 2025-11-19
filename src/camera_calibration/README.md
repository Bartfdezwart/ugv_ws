# Camera calibration

Camera calibration in python using the OpenCV library.

## Examples

Camera calibration from a set of images:
```bash
python camera_calibration.py calibrate \
   --img-dir snapshots/raw \
```

Camera calibration from a set of images and write parameters to a file:
```bash
python camera_calibration.py calibrate \
   --img-dir snapshots/raw \
   --output-file camera_params.yaml
```

Compute re-projection error from a set of images and a set of parameters:
```bash
python camera_calibration.py error \
   --img-dir snapshots/raw \
   --param-file camera_params.yaml
```

For a preview of a image, both `calibrate` and `error` have the `--test-img <IMG INDEX>`.

