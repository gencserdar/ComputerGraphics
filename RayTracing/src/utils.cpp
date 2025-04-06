#include "Scene.cpp"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h" // Texture loading

// Helper clamp for floats in [minVal, maxVal]
float clampf(float value, float minVal, float maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

// Enum for different post-processing modes
enum class PostProcessMode {
    None,        // Do nothing
    OnlyGamma,   // Only apply gamma correction
    Reinhard     // Reinhard tone mapping + gamma correction
};

// Class that handles post-processing on the final pixel buffer
class PostProcessor {
    public:
        // Constructor: sets the mode and optional gamma value (default 2.2)
        PostProcessor(PostProcessMode mode, float gammaVal = 2.2f)
            : mode(mode), gammaValue(gammaVal)
        {}

        // Applies the selected post-processing to the pixel array
        void apply(std::vector<Vec3>& pixels)
        {
            switch(mode)
            {
                case PostProcessMode::None:
                    // Do nothing
                    break;

                case PostProcessMode::OnlyGamma:
                    applyGamma(pixels, gammaValue);
                    break;

                case PostProcessMode::Reinhard:
                    // First apply tone mapping, then gamma correction
                    applyReinhardToneMapping(pixels);
                    applyGamma(pixels, gammaValue);
                    break;
            }
        }

    private:
        PostProcessMode mode;   // Selected post-process mode
        float gammaValue;       // Gamma correction value (usually 2.2)

        // Applies gamma correction to the pixel array
        void applyGamma(std::vector<Vec3>& pixels, float gammaVal)
        {
            for (auto &c : pixels)
            {
                // Convert from [0,255] to [0,1]
                float r = c.x / 255.0f;
                float g = c.y / 255.0f;
                float b = c.z / 255.0f;

                // Apply gamma correction: C' = C^(1/gamma)
                r = powf(r, 1.0f / gammaVal);
                g = powf(g, 1.0f / gammaVal);
                b = powf(b, 1.0f / gammaVal);

                // Scale back to [0,255]
                r *= 255.0f; g *= 255.0f; b *= 255.0f;

                // Clamp to valid range
                r = clampf(r, 0.0f, 255.0f);
                g = clampf(g, 0.0f, 255.0f);
                b = clampf(b, 0.0f, 255.0f);

                // Write back corrected values
                c.x = r;
                c.y = g;
                c.z = b;
            }
        }

        // Applies Reinhard tone mapping to compress HDR values to [0,1]
        void applyReinhardToneMapping(std::vector<Vec3>& pixels)
        {
            for (auto &c : pixels)
            {
                // Convert from [0,255] to [0,1]
                float r = c.x / 255.0f;
                float g = c.y / 255.0f;
                float b = c.z / 255.0f;

                // Reinhard tone mapping: C_mapped = C / (C + 1)
                r = r / (r + 1.0f);
                g = g / (g + 1.0f);
                b = b / (b + 1.0f);

                // Scale back to [0,255]
                r *= 255.0f; g *= 255.0f; b *= 255.0f;

                // Clamp to valid range
                r = clampf(r, 0.0f, 255.0f);
                g = clampf(g, 0.0f, 255.0f);
                b = clampf(b, 0.0f, 255.0f);

                // Write back tone mapped values
                c.x = r;
                c.y = g;
                c.z = b;
            }
        }
    };

    
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&t_now);

    std::ostringstream oss;
    oss << std::put_time(tm_now, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

// output PPM
void savePPM(const std::vector<Vec3>& pixels, int width, int height) {
    std::string filename = "outputImages/PPM/" + getTimestamp() + ".ppm";

    std::ofstream ofs(filename);
    ofs << "P3\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Vec3& c = pixels[y * width + x];
            float r = clampf(c.x, 0.0f, 255.0f);
            float g = clampf(c.y, 0.0f, 255.0f);
            float b = clampf(c.z, 0.0f, 255.0f);
            ofs << (int)r << " " << (int)g << " " << (int)b << " ";
        }
        ofs << "\n";
    }
    ofs.close();
    std::cout << "PPM image saved to: " << filename << std::endl;
}

// output PNG
void savePNG(const std::vector<Vec3>& pixels, int width, int height) {
    std::string filename = "outputImages/PNG/" + getTimestamp() + ".png";

    std::vector<unsigned char> data(width * height * 3);
    for (int i = 0; i < width * height; ++i) {
        data[i * 3 + 0] = static_cast<unsigned char>(clampf(pixels[i].x, 0.0f, 255.0f));
        data[i * 3 + 1] = static_cast<unsigned char>(clampf(pixels[i].y, 0.0f, 255.0f));
        data[i * 3 + 2] = static_cast<unsigned char>(clampf(pixels[i].z, 0.0f, 255.0f));
    }

    if (stbi_write_png(filename.c_str(), width, height, 3, data.data(), width * 3)) {
        std::cout << "PNG image saved to: " << filename << std::endl;
    } else {
        std::cerr << "Failed to save PNG image!" << std::endl;
    }
}
