#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <list>
#include <deque>

#include "util/arguments.h"
#include "mve/mesh.h"
#include "mve/mesh_info.h"
#include "mve/mesh_io_ply.h"
#include "mve/image.h"
#include "mve/image_io.h"
#include "math/functions.h"

#include "tex/texturing.h"

#include "simplex_noise.cpp"

typedef unsigned int uint;
constexpr float inf = std::numeric_limits<float>::infinity();

/* ---------------------------- initialize argument parser ---------------------------- */

struct AppSettings {
    std::string in_mesh;
    std::string out_prefix;
    float factor;
    int octaves;
    float persistence;
    float grid_scale;
    float noise_scale;
    float resolution;
};

AppSettings parse_args(int argc, char **argv) {
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(2);
    args.set_nonopt_minnum(2);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_MESH OUT_PREFIX");
    args.set_description("Generate noise textures for the mesh with simplex noise.");
    args.add_option('o',"octaves", true, "octaves of noise [6]");
    args.add_option('p',"persistence", true, "persistence of noise [0.6]");
    args.add_option('n',"noise_scale", true, "scale mesh coordinates [1.0f]");
    args.add_option('f',"factor", true, "scale noise [1.0f]");
    args.add_option('g',"grid_scale", true, "scale grid [5.0f]");
    args.add_option('r',"resolution", true, "sampling resolution [100.0f]");
    args.parse(argc, argv);

    AppSettings conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_prefix = args.get_nth_nonopt(1);
    conf.factor = 1.0f;
    conf.octaves = 6;
    conf.persistence = 0.6f;
    conf.grid_scale = 5.0f;
    conf.noise_scale = 1.0f;
    conf.resolution = 100.0f;

    for (util::ArgResult const* i = args.next_option();
         i != nullptr; i = args.next_option()) {
        switch (i->opt->sopt) {
        case 'f':
            conf.factor = i->get_arg<float>();
        break;
        case 'p':
            conf.persistence = i->get_arg<float>();
        break;
        case 'o':
            conf.octaves = i->get_arg<int>();
        break;
        case 'n':
            conf.noise_scale = i->get_arg<float>();
        break;
        case 'g':
            conf.grid_scale = i->get_arg<float>();
        break;
        case 'r':
            conf.resolution = i->get_arg<float>();
        break;
        default:
            throw std::invalid_argument("Invalid option");
        }
    }

    return conf;
}

void convex_hull(std::vector<math::Vec2f> const & vertices, std::list<uint> * convex_hull_ptr) {
    assert(convex_hull_ptr != nullptr);
    assert(convex_hull_ptr->empty());
    assert(vertices.size() >= 3);

    convex_hull_ptr->push_back(0);
    convex_hull_ptr->push_back(1);
    
    for (std::size_t i = 2; i < vertices.size(); ++i) {
        math::Vec2f v = vertices[i];

        std::list<uint>::iterator it = convex_hull_ptr->begin();
        for (; it != convex_hull_ptr->end(); ++it) {
            math::Vec2f v0 = vertices[*it];
            math::Vec2f v1 = std::next(it) != convex_hull_ptr->end() ?
                vertices[*std::next(it)] : vertices[convex_hull_ptr->front()];            

            math::Vec2f v01 = (v1 - v0);
            math::Vec2f n(v01[1], -v01[0]);
            float d = (v - v0).dot(n);                 
            
            if (d >= 0) break;
        }
       
        std::list<uint>::reverse_iterator rit = convex_hull_ptr->rbegin();
        for (; rit != convex_hull_ptr->rend(); ++rit) {
            math::Vec2f v0 = vertices[*rit];
            math::Vec2f v1 = rit != convex_hull_ptr->rbegin() ?
                vertices[*std::prev(rit)] : vertices[convex_hull_ptr->front()];            

            math::Vec2f v01 = (v1 - v0);
            math::Vec2f n(v01[1], -v01[0]);
            float d = (v - v0).dot(n);                 
            
            if (d >= 0) break;
        }
        
        if (it != convex_hull_ptr->end() && rit != convex_hull_ptr->rend()) {
            if (it == convex_hull_ptr->begin() && rit == convex_hull_ptr->rbegin()) {
                convex_hull_ptr->erase(it); 
            } else {
                convex_hull_ptr->erase(std::next(it), rit.base());        
            }
            convex_hull_ptr->insert(rit.base(), i);
        }
    }
}
    
void surr_rect(std::vector<math::Vec2f> const & vertices, std::vector<math::Vec2f> * surr_rect_ptr) {
    std::list<uint> hull;
    convex_hull(vertices, &hull);
   
    float smallest = inf;
 
    std::list<uint>::iterator it = hull.begin();
    for (; it != hull.end(); ++it) {
        math::Vec2f v0 = vertices[*it];
        math::Vec2f v1 = std::next(it) != hull.end() ?
            vertices[*std::next(it)] : vertices[hull.front()];            

        math::Vec2f v01n = (v1 - v0).normalized();
        math::Vec2f n = math::Vec2f(-v01n[1], v01n[0]);
        
        float h = 0.0f;
        float min = inf;
        float max = -inf;

        for (math::Vec2f const & v : vertices) {
            float d = (v - v0).dot(n);                 
            h = std::max(h, d);
            float alpha = (v - v0).dot(v01n);
            min = std::min(min, alpha);
            max = std::max(max, alpha);
        }                 
        
        float area = (max - min) * h;
        if (area < smallest)  {
            smallest = area;

            math::Vec2f a = v0 + min * v01n;
            math::Vec2f b = v0 + max * v01n;
            math::Vec2f c = b + h * n;
            math::Vec2f d = a + h * n;

            *surr_rect_ptr = {a, b, c, d};       
        }
    }
}

math::Vec3f orthogonal(math::Vec3f const & vec) {
    math::Vec3f const n0(1.0f, 0.0f, 0.0f);
    math::Vec3f const n1(0.0f, 1.0f, 0.0f);
    if (std::abs(n0.dot(vec)) < std::abs(n1.dot(vec))) {
        return n0.cross(vec);
    } else {
        return n1.cross(vec);
    }
}

/* ---------------------------------------------------------------- */

int main(int argc, char **argv) {
    AppSettings conf = parse_args(argc, argv);
    
    std::cout << "Load mesh: " << std::endl;
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = mve::geom::load_ply_mesh(conf.in_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    mve::MeshInfo mesh_info(mesh);
    mesh->ensure_normals(true, true);    

    uint const num_faces = mesh->get_faces().size() / 3;
    std::vector<math::Vec3f> const & face_normals = mesh->get_face_normals();    
    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();    
    std::vector<uint> const & faces = mesh->get_faces();    

    tex::Graph graph(num_faces);
    tex::build_adjacency_graph(mesh, mesh_info, &graph);
    
    std::vector<std::vector<uint> > segments;
    std::vector<uint8_t> assigned(num_faces, 0);
    
    std::cout << "Segmenting... " << std::flush;
    /* Segment mesh into connected planes */
    for (uint face_id = 0; face_id < num_faces; ++face_id) {
        if (assigned[face_id]) continue;

        segments.emplace_back();
        std::vector<uint> & segment = segments.back();

        math::Vec3f normal = face_normals[face_id];

        std::deque<uint> queue;
        queue.push_back(face_id);
        
        while (!queue.empty()) {
            uint face_id = queue.front(); queue.pop_front();
            if (assigned[face_id]) continue;            

            segment.push_back(face_id);
            assigned[face_id] = 255;
            
            for (std::size_t adj_face_id : graph.get_adj_nodes(face_id)) {
                if (assigned[adj_face_id]) continue;
                if (!face_normals[adj_face_id].is_similar(normal, 1e-7f)) continue;

                queue.push_back(adj_face_id);
            }
        }
    }
    std::cout << "done. Obtained " << segments.size() << " segments." << std::endl;
  
    init();
 
    tex::TexturePatches texture_patches;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        std::vector<uint> const & segment = segments[i];
        /* Project points onto common plane, parameterize and sample */
        
        math::Vec3f normal = face_normals[segment[0]];
        math::Vec3f n0 = orthogonal(normal).normalize();
        math::Vec3f n1 = n0.cross(normal).normalize();
        math::Vec3f v0 = vertices[faces[segment[0] * 3]];
        
        std::vector<math::Vec3f> ps_3d(3 * segment.size());
        std::vector<math::Vec2f> ps_2d(3 * segment.size());
        for (std::size_t j = 0; j < 3 * segment.size(); ++j) {
            uint face_id = segment[j / 3];
            uint vertex_id = faces[face_id * 3 + j % 3];
            math::Vec3f v = vertices[vertex_id];
           
            math::Vec3f & p_3d = ps_3d[j];
            p_3d = v - normal.dot(v - v0) * normal;
            
            math::Vec2f & p_2d = ps_2d[j];
            p_2d[0] = (p_3d - v0).dot(n0);
            p_2d[1] = (p_3d - v0).dot(n1);
        }

        /* Determine a surrounding rectangle of the projections */
        std::vector<math::Vec2f> rect;
        surr_rect(ps_2d, &rect);

        /* Zero area faces have no surrounding rect...*/
        if (rect.size() != 4) continue;

        /* Calculate 3d coordinates of the rectangle */
        math::Vec3f a = v0 + rect[0][0] * n0 + rect[0][1] * n1;
        math::Vec3f b = v0 + rect[1][0] * n0 + rect[1][1] * n1;
        math::Vec3f c = v0 + rect[2][0] * n0 + rect[2][1] * n1;
        math::Vec3f d = v0 + rect[3][0] * n0 + rect[3][1] * n1;

        /* Determine uv coordinates for the projections */
        math::Vec3f ab = b - a;
        math::Vec3f ad = d - a;
        v0 = a;
        n0 = ab.normalized();
        n1 = ad.normalized();

        int w = std::ceil(ab.norm() * conf.resolution);
        int h = std::ceil(ad.norm() * conf.resolution);
        
        std::cout << w << 'x' << h << std::endl;

        for (std::size_t j = 0; j < 3 * segment.size(); ++j) {
            math::Vec3f const & p_3d = ps_3d[j];
            math::Vec2f & p_2d = ps_2d[j];
            p_2d[0] = ((p_3d - v0).dot(n0) / ab.norm()) * static_cast<float>(w - 1);
            p_2d[1] = ((p_3d - v0).dot(n1) / ad.norm()) * static_cast<float>(h - 1);
        }
 
        /* Sample the volume */
        mve::ByteImage::Ptr image = mve::ByteImage::create(w, h, 3);
        math::Vec3f diag0(1.0f, 1.0f, 1.0f), diag1(-1.0f, -1.0f, 1.0f), diag2(-1.0f, 1.0f, 1.0f);
        #pragma omp parallel for        
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float fx = (x / static_cast<float>(w - 1)) * ab.norm();
                float fy = (y / static_cast<float>(h - 1)) * ad.norm();
                
                /* Calculate noise */
                math::Vec3f v = (v0 + fx * n0 + fy * n1);
                math::Vec3f sv = v * conf.noise_scale;
                float value = 0.5f + simplex_noise(sv[0], sv[1], sv[2], conf.octaves, conf.persistence) * conf.factor;
                
                /* Blend with 3D grid */
                float dist = inf;
                dist = std::min(dist, std::abs(std::fmod(v.dot(diag0), conf.grid_scale)));
                dist = std::min(dist, std::abs(std::fmod(v.dot(diag1), conf.grid_scale)));
                dist = std::min(dist, std::abs(std::fmod(v.dot(diag2), conf.grid_scale)));
                value = value * (1.0f - math::gaussian(dist, conf.grid_scale / 100.0f));
                
                value = std::min(1.0f, std::max(0.0f, value));
                image->at(x, y, 0) = value * 255.0f;                 
                image->at(x, y, 1) = value * 255.0f;                 
                image->at(x, y, 2) = value * 255.0f;                 
            }
        }        
        
        std::vector<std::size_t> faces(segment.begin(), segment.end());
        texture_patches.push_back(TexturePatch::create(i + 1, faces, ps_2d, image));
    }

    tex::TextureAtlases texture_atlases;
    tex::generate_texture_atlases(&texture_patches, &texture_atlases);
    
    tex::Model model;
    tex::build_model(mesh, texture_atlases, &model);
    tex::Model::save(model, conf.out_prefix); 
}
