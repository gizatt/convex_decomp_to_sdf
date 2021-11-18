# Mesh-to-SDF converter

![Convex decomp example](test_data/decomp_example.png)

Given a (potentially nasty, nonconvex) mesh, automatically creates an [SDF](http://sdformat.org/) that describes that object.

This has been designed to interoperate well with [Drake](drake.mit.edu), but doesn't strictly need it.

# Dependencies

For just basic conversion:

```argparse lxml numpy trimesh```

and you need `testVHACD` on your `PATH`. I use [the procedure from this script](https://github.com/mikedh/trimesh/blob/main/docker/builds/vhacd.bash) to get it.

The visualization utility requires Drake.

# Usage

```
python generate_sdf_from_mesh.py test_data/bowl_6p25in.obj --scale 0.001 --preview
```

If you have Drake installed, you can inspect the resulting SDF. Use the menu at the right to show/hide the visual and collision mesh components.

```
python inspect_sdf_in_meshcat.py test_data/bowl_6p25in.sdf
```

Both can be invoked with `--help` to see additional arguments.