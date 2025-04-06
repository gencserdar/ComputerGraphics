#include "Ray.cpp"
#include <sstream>
#include <vector>
#include "tinyxml2.cpp"

using namespace tinyxml2;

struct Camera {
    Vec3 position;
    Vec3 gaze;
    Vec3 up;
    float nearLeft, nearRight, nearBottom, nearTop;
    float nearDistance;
    int imageWidth, imageHeight;
};

struct Material {
    Vec3 ambient, diffuse, specular, mirror;
    float phongExponent;
    float textureFactor;
    int id;
};

struct MeshFace {
    int v[3];   // vertex indices
    int t[3];   // texture indices
    int n[3];   // normal indices
};

struct Mesh {
    int materialId;
    std::vector<MeshFace> faces;
};

struct PointLight {
    Vec3 position;
    Vec3 intensity;
};

struct TriangularLight {
    Vec3 v1, v2, v3;
    Vec3 intensity;
};

class Scene {
    public:
        int maxDepth;
        Vec3 backgroundColor;
        Vec3 ambientLight;

        Camera camera;
        std::vector<Vec3> vertices;
        std::vector<Vec3> normals;
        std::vector<Vec3> textureCoords; 

        std::vector<Material> materials;
        std::vector<Mesh> meshes;
        std::vector<PointLight> pointLights;
        std::vector<TriangularLight> triangleLights;

        std::string textureImagePath;

        bool loadFromXml(const std::string& filename);
    };

    static Vec3 parseVec3(const std::string& text) {
        std::stringstream ss(text);
        float x, y, z;
        ss >> x >> y >> z;
        return Vec3(x, y, z);
    }

    bool Scene::loadFromXml(const std::string& filename) {
        XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
            std::cerr << "Error loading XML file: " << filename << std::endl;
            return false; 
        }

        XMLElement* root = doc.FirstChildElement("scene");
        if (!root) return false;

        root->FirstChildElement("maxraytracedepth")->QueryIntText(&maxDepth);
        backgroundColor = parseVec3(root->FirstChildElement("background")->GetText());

        // CAMERA
        XMLElement* camElem = root->FirstChildElement("camera");
        camera.position = parseVec3(camElem->FirstChildElement("position")->GetText());
        camera.gaze = parseVec3(camElem->FirstChildElement("gaze")->GetText());
        camera.up = parseVec3(camElem->FirstChildElement("up")->GetText());

        std::stringstream ssNearPlane(camElem->FirstChildElement("nearplane")->GetText());
        ssNearPlane >> camera.nearLeft >> camera.nearRight >> camera.nearBottom >> camera.nearTop;

        camElem->FirstChildElement("neardistance")->QueryFloatText(&camera.nearDistance);

        std::stringstream ssRes(camElem->FirstChildElement("imageresolution")->GetText());
        ssRes >> camera.imageWidth >> camera.imageHeight;

        // LIGHTS
        XMLElement* lightsElem = root->FirstChildElement("lights");
        ambientLight = parseVec3(lightsElem->FirstChildElement("ambientlight")->GetText());

        for (XMLElement* p = lightsElem->FirstChildElement("pointlight"); p; p = p->NextSiblingElement("pointlight")) {
            PointLight light;
            light.position = parseVec3(p->FirstChildElement("position")->GetText());
            light.intensity = parseVec3(p->FirstChildElement("intensity")->GetText());
            pointLights.push_back(light);
        }

        for (XMLElement* t = lightsElem->FirstChildElement("triangularlight"); t; t = t->NextSiblingElement("triangularlight")) {
            TriangularLight tri;
            tri.v1 = parseVec3(t->FirstChildElement("vertex1")->GetText());
            tri.v2 = parseVec3(t->FirstChildElement("vertex2")->GetText());
            tri.v3 = parseVec3(t->FirstChildElement("vertex3")->GetText());
            tri.intensity = parseVec3(t->FirstChildElement("intensity")->GetText());
            triangleLights.push_back(tri);
        }

        // MATERIALS
        XMLElement* materialsElem = root->FirstChildElement("materials");
        for (XMLElement* m = materialsElem->FirstChildElement("material"); m; m = m->NextSiblingElement("material")) {
            Material mat;
            mat.id = std::stoi(m->Attribute("id"));
            mat.ambient = parseVec3(m->FirstChildElement("ambient")->GetText());
            mat.diffuse = parseVec3(m->FirstChildElement("diffuse")->GetText());
            mat.specular = parseVec3(m->FirstChildElement("specular")->GetText());
            mat.mirror = parseVec3(m->FirstChildElement("mirrorreflactance")->GetText());
            m->FirstChildElement("phongexponent")->QueryFloatText(&mat.phongExponent);
            m->FirstChildElement("texturefactor")->QueryFloatText(&mat.textureFactor);
            materials.push_back(mat);
        }

        // VERTEX DATA
        XMLElement* vertexElem = root->FirstChildElement("vertexdata");
        std::stringstream vstream(vertexElem->GetText());
        while (!vstream.eof()) {
            float x, y, z;
            vstream >> x >> y >> z;
            if (vstream.fail()) break;
            vertices.emplace_back(x, y, z);
        }

        // NORMALS
        XMLElement* normalElem = root->FirstChildElement("normaldata");
        std::stringstream nstream(normalElem->GetText());
        while (!nstream.eof()) {
            float x, y, z;
            nstream >> x >> y >> z;
            if (nstream.fail()) break;
            normals.emplace_back(x, y, z);
        }

        // TEXTURE COORDS
        XMLElement* texElem = root->FirstChildElement("texturedata");
        if (texElem) {
            std::stringstream tstream(texElem->GetText());
            while (!tstream.eof()) {
                float u, v;
                tstream >> u >> v;
                if (tstream.fail()) break;
                textureCoords.emplace_back(u, v, 0); // use z=0 for 2D coords
            }
        }

        XMLElement* texImg = root->FirstChildElement("textureimage");
        if (texImg)
            textureImagePath = texImg->GetText();

        // OBJECTS / MESHES
        XMLElement* objsElem = root->FirstChildElement("objects");
        for (XMLElement* meshElem = objsElem->FirstChildElement("mesh"); meshElem; meshElem = meshElem->NextSiblingElement("mesh")) {
            Mesh mesh;
            mesh.materialId = std::stoi(meshElem->FirstChildElement("materialid")->GetText());
            XMLElement* facesElem = meshElem->FirstChildElement("faces");
            std::stringstream fstream(facesElem->GetText());

            while (!fstream.eof()) {
                MeshFace face;
                for (int i = 0; i < 3; ++i) {
                    std::string token;
                    fstream >> token;
                    if (fstream.fail()) break;
                    sscanf(token.c_str(), "%d/%d/%d", &face.v[i], &face.t[i], &face.n[i]);
                    face.v[i]--; face.t[i]--; face.n[i]--; 
                }
                if (!fstream.fail())
                    mesh.faces.push_back(face);
            }
            meshes.push_back(mesh);
            std::cout << "Loaded face count: " << mesh.faces.size() << std::endl;
        }

        return true;
    }