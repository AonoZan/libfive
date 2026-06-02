#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

// Define stable C-API opaque handles and structures to match libfive
typedef void* libfive_tree;

struct libfive_vec3_t {
    float x;
    float y;
    float z;
};

extern "C" {
    // Forward-declare libfive's ABI-stable C-API functions
    libfive_tree libfive_tree_deserialize(const char* buf, size_t len);
    void libfive_tree_delete(libfive_tree t);
    float libfive_tree_eval_f(libfive_tree t, libfive_vec3_t p);

    void calculate_colors(
        float* verts, int num_verts, 
        float* matrices, 
        uint8_t* sdf_data, int* sdf_data_sizes, 
        float* sdf_colors, 
        float* blend_factors,
        float* clearance_offsets,
        int* use_shell,
        float* shell_offsets,
        int num_sdfs, 
        float* colors
    ) {
        std::vector<libfive_tree> trees;
        int offset = 0;
        
        // Deserialize trees using stable C API
        for (int i = 0; i < num_sdfs; ++i) {
            libfive_tree t = libfive_tree_deserialize(reinterpret_cast<const char*>(sdf_data) + offset, sdf_data_sizes[i]);
            trees.push_back(t);
            offset += sdf_data_sizes[i];
        }

        // Process vertices safely
        for (int i = 0; i < num_verts; ++i) {
            float vx = verts[i*3];
            float vy = verts[i*3+1];
            float vz = verts[i*3+2];

            std::vector<float> abs_dists(num_sdfs, INFINITY);
            int closest_idx = -1;
            int second_idx = -1;
            float min_dist1 = INFINITY;
            float min_dist2 = INFINITY;

            // 1. Evaluate distances to all shapes and track the two closest ones
            for (int j = 0; j < num_sdfs; ++j) {
                if (!trees[j]) continue;

                // Simple, fast manual 4x4 matrix multiplication matching column-major order
                float* m = matrices + j * 16;
                float lx = m[0]*vx + m[4]*vy + m[8]*vz + m[12];
                float ly = m[1]*vx + m[5]*vy + m[9]*vz + m[13];
                float lz = m[2]*vx + m[6]*vy + m[10]*vz + m[14];

                libfive_vec3_t p = { lx, ly, lz };
                float dist = libfive_tree_eval_f(trees[j], p);

                // Adjust distance evaluation for Shell modifier if active
                if (use_shell[j]) {
                    float thickness = std::abs(shell_offsets[j]);
                    dist = std::abs(dist) - (thickness / 2.0f);
                }

                // Adjust distance evaluation for Clearance offsets if active
                float abs_dist = std::abs(dist);
                if (clearance_offsets[j] > 1e-5f) {
                    abs_dist = std::abs(dist - clearance_offsets[j]);
                }

                abs_dists[j] = abs_dist;

                // Track closest and second-closest shapes
                if (abs_dist < min_dist1) {
                    min_dist2 = min_dist1;
                    second_idx = closest_idx;
                    
                    min_dist1 = abs_dist;
                    closest_idx = j;
                } else if (abs_dist < min_dist2) {
                    min_dist2 = abs_dist;
                    second_idx = j;
                }
            }

            float near_r = 1.0f, near_g = 1.0f, near_b = 1.0f, near_a = 1.0f;
            float second_r = 1.0f, second_g = 1.0f, second_b = 1.0f, second_a = 1.0f;

            if (closest_idx != -1) {
                near_r = sdf_colors[closest_idx*4 + 0];
                near_g = sdf_colors[closest_idx*4 + 1];
                near_b = sdf_colors[closest_idx*4 + 2];
                near_a = sdf_colors[closest_idx*4 + 3];
            }
            if (second_idx != -1) {
                second_r = sdf_colors[second_idx*4 + 0];
                second_g = sdf_colors[second_idx*4 + 1];
                second_b = sdf_colors[second_idx*4 + 2];
                second_a = sdf_colors[second_idx*4 + 3];
            }

            // 2. Determine active symmetric blend factor for this vertex intersection
            float b1 = (closest_idx != -1) ? blend_factors[closest_idx] : 0.0f;
            float b2 = (second_idx != -1) ? blend_factors[second_idx] : 0.0f;
            float w = std::max(b1, b2);

            // 3. Interpolate colors based on distance ratios
            if (w > 0.001f && closest_idx != -1 && second_idx != -1) {
                // Delta distance (how far are we from the exact transition boundary)
                float delta = abs_dists[second_idx] - abs_dists[closest_idx]; // Always >= 0
                
                // Scale factor between 0.0 (boundary) and 1.0 (edge of the blend)
                float factor = delta / w;
                if (factor > 1.0f) factor = 1.0f;
                if (factor < 0.0f) factor = 0.0f;

                // Organic cubic smoothstep curve (3t^2 - 2t^3)
                float smooth_factor = factor * factor * (3.0f - 2.0f * factor);

                // Mix ratio: 50/50 blend at the boundary, 100% closest color at the edge of the blend
                float mix = 0.5f + 0.5f * smooth_factor;

                colors[i * 4 + 0] = near_r * mix + second_r * (1.0f - mix);
                colors[i * 4 + 1] = near_g * mix + second_g * (1.0f - mix);
                colors[i * 4 + 2] = near_b * mix + second_b * (1.0f - mix);
                colors[i * 4 + 3] = near_a * mix + second_a * (1.0f - mix);
            } else {
                // Perfectly sharp cutoff edge
                colors[i * 4 + 0] = near_r;
                colors[i * 4 + 1] = near_g;
                colors[i * 4 + 2] = near_b;
                colors[i * 4 + 3] = near_a;
            }
        }

        // Safely deallocate trees using C API
        for (auto t : trees) {
            if (t) {
                libfive_tree_delete(t);
            }
        }
    }
}