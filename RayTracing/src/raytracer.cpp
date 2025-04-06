#include <sstream>
#include <thread>
#include <functional>
#include <algorithm>
#include <cmath>
#include "utils.cpp" 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h" // Texture loading

using namespace std;

class RayTracer {
public:
    RayTracer(const Scene& scene) : scene(scene) {
        // Load texture if specified
        if (!scene.textureImagePath.empty()) {
            int n;
            textureData = stbi_load(scene.textureImagePath.c_str(), &texWidth, &texHeight, &n, 3);
            if (!textureData) {
                std::cerr << "Failed to load texture: " << scene.textureImagePath << std::endl;
            }
        }
    }

    ~RayTracer() {
        if (textureData) {
            stbi_image_free(textureData);
        }
    }

    // Performs bilinear interpolation sampling from a texture (in [0..255] range)
    Vec3 sampleBilinear(float u, float v) {
        // Clamp texture coordinates to [0, 1]
        u = clampf(u, 0.0f, 1.0f);
        v = clampf(v, 0.0f, 1.0f);

        // Convert normalized coordinates to texture space
        float texX = u * (texWidth - 1);
        // Flip v
        float texY = (1.0f - v) * (texHeight - 1);

        // Find surrounding pixel indices
        int x0 = static_cast<int>(texX);
        int y0 = static_cast<int>(texY);
        int x1 = std::min(x0 + 1, texWidth - 1);
        int y1 = std::min(y0 + 1, texHeight - 1);

        // Compute the fractional parts
        float fracX = texX - x0;
        float fracY = texY - y0;

        // Helper function to fetch RGB color at a pixel
        auto getColorAt = [&](int px, int py) {
            int index = (py * texWidth + px) * 3;
            return Vec3(
                static_cast<float>(textureData[index]),
                static_cast<float>(textureData[index + 1]),
                static_cast<float>(textureData[index + 2])
            );
        };

        // Sample the 4 neighboring pixels
        Vec3 color00 = getColorAt(x0, y0); // Top-left
        Vec3 color10 = getColorAt(x1, y0); // Top-right
        Vec3 color01 = getColorAt(x0, y1); // Bottom-left
        Vec3 color11 = getColorAt(x1, y1); // Bottom-right

        // Interpolate horizontally
        Vec3 interpTop = color00 * (1.0f - fracX) + color10 * fracX;
        Vec3 interpBottom = color01 * (1.0f - fracX) + color11 * fracX;

        // Interpolate vertically and return final color
        return interpTop * (1.0f - fracY) + interpBottom * fracY; // Still in [0..255] range
    }


    // Main recursive ray trace
    Vec3 traceRay(const Ray& ray, int depth = 0) {
        // Stop recursion if maximum depth exceeded
        if (depth > scene.maxDepth) {
            return scene.backgroundColor; 
        }

        float tMin = 1e30f;                // Closest intersection distance
        int hitMatId = -1;                 // ID of the material at intersection
        Vec3 hitPoint, hitNormal, hitUV;   // Intersection data

        // Find nearest intersection
        for (const auto& mesh : scene.meshes) {
            for (const auto& face : mesh.faces) {
                // Get triangle vertices from indices
                Vec3 v0 = scene.vertices[face.v[0]];
                Vec3 v1 = scene.vertices[face.v[1]];
                Vec3 v2 = scene.vertices[face.v[2]];

                float t, u, v;
                // Check if the ray intersects the current triangle (v0, v1, v2)
                // If so, it returns the distance `t` and barycentric coordinates `u`, `v`
                if (intersectTriangle(ray, v0, v1, v2, t, u, v)) {
                    // If this intersection is closer than any previously found hit
                    if (t < tMin) {
                        tMin = t;  // Update the closest hit distance
                        hitMatId = mesh.materialId; // Save material ID of the hit triangle
                        hitPoint = ray.at(t); // Calculate the 3D point where the ray hits

                        // === Interpolated normal for smooth shading ===
                        // Get normals at the triangle’s 3 vertices
                        Vec3 n0 = scene.normals[face.n[0]];
                        Vec3 n1 = scene.normals[face.n[1]];
                        Vec3 n2 = scene.normals[face.n[2]];
                        
                        // Compute barycentric weight for the 3rd vertex
                        float w = 1.0f - u - v;
                        
                        // Interpolate the normals using barycentric weights (w, u, v)
                        // This creates a smooth normal across the surface (Gouraud/Phong-style shading)
                        hitNormal = (n0 * w + n1 * u + n2 * v).normalize();

                        // === Interpolated texture coordinates (UVs) ===
                        // Get UV coordinates at each vertex of the triangle
                        Vec3 uv0 = scene.textureCoords[face.t[0]];
                        Vec3 uv1 = scene.textureCoords[face.t[1]];
                        Vec3 uv2 = scene.textureCoords[face.t[2]];
                        
                        // Interpolate UVs using same barycentric weights
                        // This gives the exact texture coordinate at the hit point
                        hitUV = uv0 * w + uv1 * u + uv2 * v;
                    }
                }
            }
        }


        // If we hit something, do shading
        if (hitMatId != -1) {
            // Lookup the material based on hit material ID
            const Material* mat = nullptr;
            for (auto &m : scene.materials) {
                if (m.id == hitMatId) {
                    mat = &m;
                    break;
                }
            }

            // Fallback: if material not found, return background color
            if (!mat) {
                return scene.backgroundColor;
            }

            // Default texture color (white, no texture influence)
            Vec3 textureColor(255.0f, 255.0f, 255.0f); 

            // If texture is available, sample it using UV coordinates
            if (textureData) {
                textureColor = sampleBilinear(hitUV.x, hitUV.y);
            }

            float tf = mat->textureFactor; // Texture influence factor

            // If textureFactor is 1.0, use pure texture color with no shading
            if (fabs(tf - 1.0f) < 1e-6) {
                return textureColor; 
            }

            // ----------- AMBIENT LIGHTING -----------

            // 1) Interpolate ambient color between material and texture
            Vec3 blendedAmbient = mat->ambient * (1.0f - tf) + textureColor * tf;

            // 2) Multiply with global ambient light color
            Vec3 color = Vec3(
                scene.ambientLight.x * blendedAmbient.x,
                scene.ambientLight.y * blendedAmbient.y,
                scene.ambientLight.z * blendedAmbient.z
            );

            // ----------- DIRECT LIGHTING -----------

            // Interpolated diffuse color (material + texture blend)
            Vec3 blendedDiffuse = mat->diffuse * (1.0f - tf) + textureColor * tf;

            Vec3 N = hitNormal;                   // Surface normal
            Vec3 V = -1 * ray.direction.normalize(); // View direction

            Vec3 totalLighting(0.0f, 0.0f, 0.0f);  // Total contribution from all lights

            // ---------- POINT LIGHTS ----------
            for (auto &light : scene.pointLights) {
                Vec3 Lvec = light.position - hitPoint; // Vector from hit point to light
                float dist = Lvec.length();            // Distance to light
                Vec3 Ldir = Lvec / dist;               // Light direction (normalized)

                // --- Shadow check ---
                Ray shadowRay(hitPoint + N * 1e-4f, Ldir); //+ N * 1e-4f this part is to prevent self-intersection problem
                bool inShadow = false;
                // Again we do the what we do at the start of this function
                for (auto &mesh : scene.meshes) {
                    for (auto &face : mesh.faces) {
                        float tt, uu, vv;
                        if (intersectTriangle(shadowRay,
                                            scene.vertices[face.v[0]],
                                            scene.vertices[face.v[1]],
                                            scene.vertices[face.v[2]],
                                            tt, uu, vv)) 
                        {
                            if (tt > 1e-4f && tt < dist) {
                                inShadow = true;
                                break;
                            }
                        }
                    }
                    if (inShadow) break;
                }
                if (inShadow) continue; // Skip light contribution if in shadow

                // --- Diffuse (Lambertian) ---
                float diffFactor = std::max(0.0f, N.dot(Ldir));

                // --- Specular (Phong) ---
                float specFactor = 0.0f;
                if (diffFactor > 0.0f) {
                    Vec3 R = (N * (2.0f * (N.dot(Ldir))) - Ldir).normalize(); // Reflect direction
                    float rv = std::max(0.0f, R.dot(V));
                    specFactor = powf(rv, mat->phongExponent);
                }

                // Light attenuation by distance squared
                float attenuation = 1.0f / (dist * dist);
                Vec3 lightInt = light.intensity * attenuation;

                // Final diffuse and specular contributions from this light
                Vec3 diffPart = blendedDiffuse * diffFactor;
                Vec3 specPart = mat->specular * specFactor;

                totalLighting += Vec3(
                    (diffPart.x + specPart.x) * lightInt.x,
                    (diffPart.y + specPart.y) * lightInt.y,
                    (diffPart.z + specPart.z) * lightInt.z
                );
            }

            // ---------- TRIANGLE LIGHTS (Directional lights) ----------
            for (auto &triLight : scene.triangleLights) {
                // Compute direction from triangle face (v1 → v2 x v3)
                Vec3 crossDir = (triLight.v2 - triLight.v1).cross(triLight.v3 - triLight.v1);
                Vec3 Ldir = crossDir.normalize();         // Light direction
                Vec3 fromLight = -1 * Ldir;               // Direction from light towards surface

                // --- Shadow check (directional lights have infinite distance) ---
                Ray shadowRay(hitPoint + N * 1e-4f, fromLight);
                bool inShadow = false;
                for (auto &mesh : scene.meshes) {
                    for (auto &face : mesh.faces) {
                        float tt, uu, vv;
                        if (intersectTriangle(shadowRay,
                                            scene.vertices[face.v[0]],
                                            scene.vertices[face.v[1]],
                                            scene.vertices[face.v[2]],
                                            tt, uu, vv)) 
                        {
                            if (tt > 1e-4f) {
                                inShadow = true;
                                break;
                            }
                        }
                    }
                    if (inShadow) break;
                }
                if (inShadow) continue;

                // --- Diffuse lighting ---
                float diffFactor = std::max(0.0f, N.dot(fromLight));

                // --- Specular lighting ---
                float specFactor = 0.0f;
                if (diffFactor > 0.0f) {
                    Vec3 R = (N * (2.0f * (N.dot(fromLight))) - fromLight).normalize();
                    float rv = std::max(0.0f, R.dot(V));
                    specFactor = powf(rv, mat->phongExponent);
                }

                // Final diffuse and specular contributions from this directional light
                Vec3 diffPart = blendedDiffuse * diffFactor;
                Vec3 specPart = mat->specular * specFactor;

                totalLighting += Vec3(
                    (diffPart.x + specPart.x) * triLight.intensity.x,
                    (diffPart.y + specPart.y) * triLight.intensity.y,
                    (diffPart.z + specPart.z) * triLight.intensity.z
                );
            }

            // ----------- REFLECTIONS (Mirror Materials) -----------

            Vec3 localColor = color + totalLighting; // Ambient + lighting = base color

            // If material is reflective and we haven't hit recursion limit
            if (depth < scene.maxDepth) {
                if (mat->mirror.x > 0.0f || mat->mirror.y > 0.0f || mat->mirror.z > 0.0f) {
                    // Compute reflection direction
                    Vec3 reflectDir = ray.direction - N * 2.0f * ray.direction.dot(N);
                    reflectDir = reflectDir.normalize();

                    // Trace reflected ray slightly offset to avoid self-intersection
                    Ray reflectedRay(hitPoint + N * 1e-4f, reflectDir);
                    Vec3 reflectedColor = traceRay(reflectedRay, depth + 1);

                    // Blend local color with reflected color based on mirror reflectivity
                    localColor = localColor * (Vec3(1,1,1) - mat->mirror) + reflectedColor * mat->mirror;
                }
            }

            // Return final computed color
            return localColor;
        }

        // If nothing was hit, return background color
        return scene.backgroundColor;
    }

    void render(int width, int height, std::vector<Vec3>& pixels) {
        // Camera setup
        Vec3 camPos = scene.camera.position;
        Vec3 camDir = scene.camera.gaze.normalize();
        Vec3 camRight = camDir.cross(scene.camera.up).normalize();
        Vec3 camUp = camRight.cross(camDir).normalize();
    
        float l = scene.camera.nearLeft;
        float r = scene.camera.nearRight;
        float b = scene.camera.nearBottom;
        float t = scene.camera.nearTop;
        float d = scene.camera.nearDistance;
    
        // Resize output buffer
        pixels.resize(width * height);
    
        // Decide how many threads we want to use
        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) {
            numThreads = 4; // Fallback if hardware_concurrency() is not available
        }
    
        // We’ll break the image rows into chunks
        int chunkSize = height / numThreads;
        int remainder = height % numThreads;
    
        // A lambda function that renders rows [startRow, endRow)
        auto renderRows = [&](int startRow, int endRow) {
            for (int y = startRow; y < endRow; ++y) {
                for (int x = 0; x < width; ++x) {
                    float uu = l + (r - l) * (x + 0.5f) / width;
                    float vv = t - (t - b) * (y + 0.5f) / height;
    
                    Vec3 pixelWorld = camPos + camDir * d + camRight * uu + camUp * vv;
                    Vec3 rayDir = (pixelWorld - camPos).normalize();
                    Ray ray(camPos, rayDir);
    
                    // Perform the ray trace and store it
                    pixels[y * width + x] = traceRay(ray, 0);
                }
            }
        };
    
        // Create threads
        std::vector<std::thread> threads;
        threads.reserve(numThreads);
    
        // Assign row ranges to each thread
        int currentStart = 0;
        for (unsigned int i = 0; i < numThreads; ++i) {
            int currentEnd = currentStart + chunkSize + ((int)i < remainder ? 1 : 0);
            threads.emplace_back(renderRows, currentStart, currentEnd);
            currentStart = currentEnd;
        }
    
        // Wait for all threads to finish
        for (auto &th : threads) {
            th.join();
        }
    }

private:
    const Scene& scene;
    unsigned char* textureData = nullptr;
    int texWidth = 0, texHeight = 0;

    // Moller-Trumbore triangle-ray intersection algorithm
    bool intersectTriangle(const Ray& ray,
        const Vec3& v0, const Vec3& v1, const Vec3& v2,
        float& t, float& outU, float& outV)
    {
    // Calculate triangle edges
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;

    // Compute the determinant
    Vec3 h = ray.direction.cross(edge2);
    float a = edge1.dot(h);

    // If the determinant is close to 0, the ray is parallel to the triangle
    if (fabs(a) < 1e-8f) return false;

    // Invert the determinant
    float f = 1.0f / a;

    // Distance from v0 to ray origin
    Vec3 s = ray.origin - v0;

    // Calculate barycentric coordinate u
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) return false; // Outside the triangle

    // Calculate barycentric coordinate v
    Vec3 q = s.cross(edge1);
    float v = f * ray.direction.dot(q);
    if (v < 0.0f || u + v > 1.0f) return false; // Outside the triangle

    // At this stage, we can compute t (ray parameter for intersection point)
    float tempT = f * edge2.dot(q);

    // Check if the intersection is in the positive ray direction
    if (tempT < 1e-4f) return false; // Ignore very close or behind intersections

    // Store results
    t = tempT;     // Distance from ray origin to intersection point
    outU = u;      // Barycentric u coordinate
    outV = v;      // Barycentric v coordinate

    return true;   // Intersection found
    }

};

int main() {
    Scene scene;
    if (!scene.loadFromXml("scene.xml")) {
        std::cerr << "Failed to load scene." << std::endl;
        return 1;
    }

    int w = scene.camera.imageWidth;
    int h = scene.camera.imageHeight;
    std::vector<Vec3> pixels;

    RayTracer tracer(scene);

    auto start = std::chrono::high_resolution_clock::now(); 
    tracer.render(w, h, pixels);
    auto end = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Render time: " << elapsed.count() << " seconds\n";

    //I prefer these texture factor settings for each Post Processor Mode:
    //If Reinhard or OnlyGamma: 0.5 texture factor 
    //If None: 0.02 texture factor
    //I love no postprocessing with 0.02 texture factor setting if there are not too much light
    //But Reinhard does perfect job when it comes to shading and high intensity light, overall I think Reinhard is the best
    PostProcessor ppMode(PostProcessMode::Reinhard, 2.0f); //options: None, OnlyGamma, Reinhard
    ppMode.apply(pixels);

    savePPM(pixels, w, h);
    savePNG(pixels, w, h);

    return 0;
}
