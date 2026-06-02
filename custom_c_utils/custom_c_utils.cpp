#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Define stable C-API opaque handles and structures to match libfive
typedef void* libfive_tree;

struct libfive_vec3_t {
    float x;
    float y;
    float z;
};

// Fold a linear coordinate safely back to the base primitive coordinate system
float fold_linear(float val, int count, float spacing, float child_pos) {
    if (count > 1 && std::abs(spacing) > 1e-5f) {
        // Shift using the child's original coordinate to align with the array span
        float val_shifted = val + child_pos;
        
        // Find closest instance index
        int idx = std::round(val_shifted / spacing);
        if (idx < 0) idx = 0;
        if (idx >= count) idx = count - 1;
        
        // Fold back to the original child shape's position (located at the end of the array, index count - 1)
        return val - (idx - (count - 1)) * spacing;
    }
    return val;
}

// Fold a radial coordinate back to the child shape's original angle
void fold_radial(float& lx, float& ly, int count, float cx, float cy, float child_x, float child_y) {
    if (count > 1) {
        float dx = lx - cx;
        float dy = ly - cy;
        float r = std::sqrt(dx*dx + dy*dy);
        if (r > 1e-5f) {
            float theta = std::atan2(dy, dx);
            
            // Calculate child's original angle relative to the pivot
            float theta_child = std::atan2(child_y - cy, child_x - cx);
            
            float theta_rel = theta - theta_child;
            
            // Wrap relative angle to [-pi, pi] to find the closest segment safely
            while (theta_rel < -M_PI) theta_rel += 2.0f * M_PI;
            while (theta_rel > M_PI)  theta_rel -= 2.0f * M_PI;
            
            float sector = (2.0f * M_PI) / count;
            int idx = std::round(theta_rel / sector);
            
            float theta_folded = theta_rel - (idx * sector) + theta_child;
            
            lx = r * std::cos(theta_folded) + cx;
            ly = r * std::sin(theta_folded) + cy;
        }
    }
}

extern "C" {
    // Forward-declare libfive's ABI-stable C-API functions
    libfive_tree libfive_tree_deserialize(const char* buf, size_t len);
    void libfive_tree_delete(libfive_tree t);
    float libfive_tree_eval_f(libfive_tree t, libfive_vec3_t p);

    void calculate_colors(
        float* verts, int num_verts, 
        float* matrices, 
        float* child_matrices, 
        uint8_t* sdf_data, int* sdf_data_sizes, 
        float* sdf_colors, 
        float* blend_factors,
        float* clearance_offsets,
        int* use_shell,
        float* shell_offsets,

        int* array_modes,
        int* array_counts_x,
        int* array_counts_y,
        int* array_counts_z,
        float* array_spacings_x,
        float* array_spacings_y,
        float* array_spacings_z,
        float* array_shifts_x,
        float* array_shifts_y,
        float* array_shifts_z,
        int* radial_counts,
        float* radial_centers_x,
        float* radial_centers_y,
        float* radial_children_x,
        float* radial_children_y,

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

                // Map vertex from World Space to Group local space
                float* m_grp = matrices + j * 16;
                float gx = m_grp[0]*vx + m_grp[4]*vy + m_grp[8]*vz + m_grp[12];
                float gy = m_grp[1]*vx + m_grp[5]*vy + m_grp[9]*vz + m_grp[13];
                float gz = m_grp[2]*vx + m_grp[6]*vy + m_grp[10]*vz + m_grp[14];

                // Fold coordinate space inside the Group's space if array modifiers are active
                int mode = array_modes[j];
                if (mode == 1) { // LINEAR
                    gx = fold_linear(gx, array_counts_x[j], array_spacings_x[j], array_shifts_x[j]);
                    gy = fold_linear(gy, array_counts_y[j], array_spacings_y[j], array_shifts_y[j]);
                    gz = fold_linear(gz, array_counts_z[j], array_spacings_z[j], array_shifts_z[j]);
                } else if (mode == 2) { // RADIAL
                    fold_radial(gx, gy, radial_counts[j], radial_centers_x[j], radial_centers_y[j], radial_children_x[j], radial_children_y[j]);
                }

                // Map vertex from Group Space to Child local space
                float* m_ch = child_matrices + j * 16;
                float lx = m_ch[0]*gx + m_ch[4]*gy + m_ch[8]*gz + m_ch[12];
                float ly = m_ch[1]*gx + m_ch[5]*gy + m_ch[9]*gz + m_ch[13];
                float lz = m_ch[2]*gx + m_ch[6]*gy + m_ch[10]*gz + m_ch[14];

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