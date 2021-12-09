import argparse
import logging
from lxml import etree as ET
import numpy as np
import os
import trimesh
import sys


def do_visual_mesh_simplification(input_mesh_path, target_tris=1000):
    '''
    Given an obj path, simplifies the geometry to make it easier to render
    by creating a mesh alongside it with a "_simple_vis.<ext>" postfix.
    - Looks for a texture file at a fixed relative path
    
    Args:
    - input_mesh_path: String path to mesh file. Only 'obj' format is tested,
        but others might work.
    - target_tris: Currently unusued, but would be target for mesh decimation.
    Returns:
    - output obj file path
    '''
    import open3d
    import cv2
    import imutils

    # TODO(gizatt) What gives, open3d is trashing my models...
    logging.warning("As of writing, this is sometimes creating terrible models.")

    mesh_minus_ext, mesh_ext = os.path.splitext(input_mesh_path)
    output_mesh_path = mesh_minus_ext + "_simple_vis" + mesh_ext
    
    mesh = open3d.io.read_triangle_mesh(input_mesh_path)
    mesh.compute_vertex_normals()
    simplified_mesh = mesh.simplify_quadric_decimation(target_tris)
    simplified_mesh.compute_vertex_normals()
    open3d.io.write_triangle_mesh(output_mesh_path, simplified_mesh)
    return output_mesh_path


def do_collision_mesh_simplification(mesh, mesh_name, mesh_dir, preview_with_trimesh=False, **kwargs):
    '''
    Given a mesh, performs a convex decomposition of it with
    trimesh _ vhacd, saving all the parts in a subfolder named
    `<mesh_filename>_parts`.

    Args:
    - input_mesh_path: String path to mesh file to decompose. Only
        'obj' format is currently tested, but other formats supported
        by trimesh might work.
    - preview_with_trimesh: Whether to open (and block on) a window to preview
    the decomposition.
    - A set of control kwargs, plus any additional kwargs, are passed to the convex
      decomposition routine 'vhacd'; you can run `testVHACD --help` to see options.

    Returns:
    - List of generated mesh file parts, in obj format.
    '''

    # Create a subdir for the convex decomp parts.
    mesh_parts_folder = mesh_name + "_parts"
    out_dir = os.path.join(mesh_dir, mesh_parts_folder)
    os.makedirs(out_dir, exist_ok=True)

    if preview_with_trimesh:
        logging.info("Showing mesh before decomp. Close window to proceed.")
        mesh.show()
    try:
        convex_pieces = []
        convex_pieces_new = trimesh.decomposition.convex_decomposition(
            mesh, **kwargs)
        if not isinstance(convex_pieces_new, list):
            convex_pieces_new = [convex_pieces_new]
        convex_pieces += convex_pieces_new
    except Exception as e:
        logging.error("Problem performing decomposition: %s", e)
    
    if preview_with_trimesh:
        # Display the convex decomp, giving each a random colors
        # to make them easier to distinguish.
        for part in convex_pieces:
            this_color = trimesh.visual.random_color()
            part.visual.face_colors[:] = this_color
        scene = trimesh.scene.scene.Scene()
        for part in convex_pieces:
            scene.add_geometry(part)
        
        logging.info(
            "Showing mesh convex decomp into %d parts. Close window to proceed."
            % (len(convex_pieces))
        )
        scene.show()

    out_paths = []
    for k, part in enumerate(convex_pieces):
        piece_name = '%s_convex_piece_%03d.obj' % (mesh_name, k)
        full_path = os.path.join(out_dir, piece_name)
        trimesh.exchange.export.export_mesh(part, full_path)
        out_paths.append(full_path)
    return out_paths


def calc_mesh_inertia(mesh, density=2000):
    '''
    Given a mesh, calculates its total mass and inertia assuming
    a fixed density.
    Args:
    - mesh: A trimesh mesh.
    - density: Density of object in kg/m^3, used for inertia calculation.
    Returns: (mass, inertia)
    - out_paths: List of generated mesh file parts, in obj format.
    - inertia: total inertia of the input mesh.
    '''
    mesh.density = density
    I = mesh.moment_inertia
    return mesh.mass, mesh.moment_inertia


def create_sdf_with_convex_decomp(input_mesh_path, scale=1., do_visual_simplification=False, target_tris=1000, preview_with_trimesh=False, density=2000, **kwargs):
    '''
    Given an input mesh file, produces an SDF if the same directory that:
    - Uses the mesh as its sole visual geometry.
    - Performs a convex decomposition of the mesh, and uses those pieces
    as the collision geometry.
    - Inserts inertia for the object, calculated from the original
    mesh assuming a constant density.

    The SDF is saved as `<mesh_file>.sdf` next to the mesh file.

    Args:
    - input_mesh_path: Path to the mesh file
    - preview_with_trimesh: Whether to show 3D previews of pre/post decomposition.
    - density: Assumed density of the object, in kg/m^3.
    - kwargs: Passed through to do_collision_mesh_simplification as convex decomp args.
    '''


    # Get mesh name.
    dir_path, mesh_filename = os.path.split(input_mesh_path)
    mesh_minus_ext, _ = os.path.splitext(mesh_filename)
    sdf_path = os.path.join(dir_path, mesh_minus_ext + ".sdf")

    # Load in and prescale mesh.
    mesh = trimesh.load(input_mesh_path, skip_materials=True)
    mesh.apply_scale(scale)
    
    # Generate SDF file and the robot and link elements.
    robot_name = mesh_minus_ext
    root_item = ET.Element('sdf', version="1.5", nsmap={'drake': 'drake.mit.edu'})
    model_item = ET.SubElement(root_item, 'model', name=robot_name)
    link_name = "{}_body_link".format(robot_name)
    link_item = ET.SubElement(model_item, "link", name=link_name)
    pose_item = ET.SubElement(link_item, "pose")
    pose_item.text = "0 0 0 0 0 0"

    # Set up object inertia.
    mass, I = calc_mesh_inertia(mesh, density=density)
    inertial_item = ET.SubElement(link_item, "inertial")
    mass_item = ET.SubElement(inertial_item, 'mass')
    mass_item.text = '{:.4E}'.format(mass)
    inertia_item = ET.SubElement(inertial_item, "inertia")
    for i in range(3):
        for j in range(i, 3):
            item = ET.SubElement(inertia_item, "i" + "xyz"[i] + "xyz"[j])
            item.text = '{:.4E}'.format(I[i, j])

    # Set up object visual geometry.
    if do_visual_simplification:
        visual_mesh_filename = do_visual_mesh_simplification(input_mesh_path, target_tris=target_tris)
    else:
        visual_mesh_filename = input_mesh_path
    logging.warn("%s -> %s", visual_mesh_filename, sdf_path)
    visual_mesh_filename = os.path.relpath(visual_mesh_filename, dir_path)
    visual_item = ET.SubElement(link_item, "visual", name="visual")
    geometry_item = ET.SubElement(visual_item, 'geometry')
    mesh_item = ET.SubElement(geometry_item, 'mesh')
    uri_item = ET.SubElement(mesh_item, "uri")
    uri_item.text = visual_mesh_filename
    scale_item = ET.SubElement(mesh_item, "scale")
    scale_item.text = "{:.4E} {:.4E} {:.4E}".format(scale, scale, scale)

    # Set up object collision geometry.
    collision_paths = do_collision_mesh_simplification(mesh, mesh_name=mesh_minus_ext, mesh_dir=dir_path, preview_with_trimesh=preview_with_trimesh, **kwargs)
    for k, new_model_path in enumerate(collision_paths):
        new_model_path = os.path.relpath(new_model_path, dir_path)
        # Create a new XML subtree for the collection of meshes
        # we just created. I *think* each convex piece needs
        # to be in its own collision tag, otherwise Drake
        # seems to be ignoring them...
        collision_item = ET.SubElement(link_item, 'collision', name="collision_%04d" % k)
        geometry_item = ET.SubElement(collision_item, 'geometry')
        mesh_item = ET.SubElement(geometry_item, "mesh")
        uri_item = ET.SubElement(mesh_item, "uri")
        uri_item.text = new_model_path
        ET.SubElement(mesh_item, '{drake.mit.edu}declare_convex')

    logging.info("Writing SDF to %s" % sdf_path)
    ET.ElementTree(root_item).write(sdf_path, pretty_print=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate SDF from a mesh.')
    parser.add_argument('mesh_file', type=str, help='Path to mesh file.')
    parser.add_argument('--preview', default=False, action="store_true",
                        help="Preview decomp with a trimesh window?")
    parser.add_argument('--scale', type=float, default=1.,
                        help="Scale factor to convert the specified mesh's coordinates to meters.")
    parser.add_argument('--density', type=float, default=2000,
                        help="Assumed density in kg/m^3 of object, for inertia calculation.")
    parser.add_argument('--do_visual_simplification', default=False, action="store_true",
                        help="Do additional visual simplification of mesh. Requires open3d. Probably won't preserve materials.")
    parser.add_argument('--target_tris', type=int, default=1000,
                        help="If we do visual simplification, we decimate to this # of triangles.")
    parser.add_argument('--resolution', type=int, default=100000,
                        help="VHACD voxel resolution.")
    parser.add_argument('--maxhulls', type=int, default=12,
                        help="VHACD max # of convex hulls.")
    parser.add_argument('--minVolumePerCH', type=float, default=0.001,
                        help="VHACD min convex hull volume.")
    parser.add_argument('--maxNumVerticesPerCH', type=int, default=12,
                        help="VHACD voxel resolution.")
    parser.add_argument('--loglevel', type=str, default="INFO",
                        choices=["CRITICAL", "ERROR",  "WARNING", "INFO", "DEBUG"],
                        help='Log level.')

    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel)

    mesh_path = os.path.abspath(args.mesh_file)
    if not os.path.exists(mesh_path):
        logging.error("No mesh found at %s" % mesh_path)
        sys.exit(-1)

    create_sdf_with_convex_decomp(
        mesh_path, scale=args.scale, do_visual_simplification=args.do_visual_simplification,
        target_tris=args.target_tris, preview_with_trimesh=args.preview, density=args.density,
        resolution=args.resolution, maxhulls=args.maxhulls, maxNumVerticesPerCH=args.maxNumVerticesPerCH,
        minVolumePerCH=args.minVolumePerCH, pca=1
    )
