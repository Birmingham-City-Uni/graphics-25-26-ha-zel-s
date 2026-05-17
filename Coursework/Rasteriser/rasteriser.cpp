#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cstdint>
#include <array>

#include <lodepng.h>
#include <Eigen/Dense>

#include "Image.hpp"
#include "LinAlg.hpp"
#include "Light.hpp"
#include "Mesh.hpp"

struct Triangle {
    std::array<Eigen::Vector3f, 3> screen;
    std::array<Eigen::Vector3f, 3> verts;
    std::array<Eigen::Vector3f, 3> norms;
    std::array<Eigen::Vector2f, 3> texs;
    std::array<float, 3> clipW;
    std::array<Eigen::Vector4f, 3> ndc;
};

Eigen::Matrix4f projectionMatrix(
    int height,
    int width,
    float horzFov = 70.f * M_PI / 180.f,
    float zFar = 100.f,
    float zNear = 1.0f)
{
    float aspect = static_cast<float>(width) / static_cast<float>(height);
    float vertFov = 2.0f * std::atan(std::tan(horzFov * 0.5f) / aspect);

    float f = 1.0f / std::tan(vertFov * 0.5f);

    Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
    p(0, 0) = f / aspect;
    p(1, 1) = f;
    p(2, 2) = (zFar + zNear) / (zNear - zFar);
    p(2, 3) = (2.0f * zFar * zNear) / (zNear - zFar);
    p(3, 2) = -1.0f;

    return p;
}

void findScreenBoundingBox(const Triangle& t, int width, int height,
    int& minX, int& minY, int& maxX, int& maxY)
{
    float min_x = t.screen[0].x();
    float min_y = t.screen[0].y();
    float max_x = min_x;
    float max_y = min_y;

    for (int i = 1; i < 3; ++i) {
        min_x = std::min(min_x, t.screen[i].x());
        min_y = std::min(min_y, t.screen[i].y());
        max_x = std::max(max_x, t.screen[i].x());
        max_y = std::max(max_y, t.screen[i].y());
    }

    minX = static_cast<int>(std::floor(min_x));
    minY = static_cast<int>(std::floor(min_y));
    maxX = static_cast<int>(std::ceil(max_x));
    maxY = static_cast<int>(std::ceil(max_y));

    minX = std::clamp(minX, 0, width - 1);
    minY = std::clamp(minY, 0, height - 1);
    maxX = std::clamp(maxX, 0, width - 1);
    maxY = std::clamp(maxY, 0, height - 1);
}

void drawTriangle(
    std::vector<uint8_t>& image,
    int width,
    int height,
    std::vector<float>& zBuffer,
    const Triangle& t,
    const std::vector<std::unique_ptr<Light>>& lights,
    const std::vector<uint8_t>& albedoTexture,
    int texWidth,
    int texHeight,
    const Eigen::Vector3f& cameraPos)
{
    int minX, minY, maxX, maxY;
    findScreenBoundingBox(t, width, height, minX, minY, maxX, maxY);

    auto edgeFunc = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c) {
        return (c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x());
    };

    const Eigen::Vector2f s0 = t.screen[0].head<2>();
    const Eigen::Vector2f s1 = t.screen[1].head<2>();
    const Eigen::Vector2f s2 = t.screen[2].head<2>();

    float area = edgeFunc(s0, s1, s2);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            Eigen::Vector2f p(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);

            float w0 = edgeFunc(s1, s2, p);
            float w1 = edgeFunc(s2, s0, p);
            float w2 = edgeFunc(s0, s1, p);

            if (w0 * area < 0.0f || w1 * area < 0.0f || w2 * area < 0.0f)
                continue;

            float b0 = w0 * invArea;
            float b1 = w1 * invArea;
            float b2 = w2 * invArea;

            int idx = y * width + x;

            float z0 = t.ndc[0].z() / t.clipW[0];
            float z1 = t.ndc[1].z() / t.clipW[1];
            float z2 = t.ndc[2].z() / t.clipW[2];

            float depth = (b0 * z0 + b1 * z1 + b2 * z2) /
                (b0 / t.clipW[0] + b1 / t.clipW[1] + b2 / t.clipW[2]);

            depth = (depth + 1.0f) * 0.5f;

            if (depth > zBuffer[idx]) continue;
            zBuffer[idx] = depth;

            float pw0 = b0 / t.clipW[0];
            float pw1 = b1 / t.clipW[1];
            float pw2 = b2 / t.clipW[2];
            float invW = pw0 + pw1 + pw2;

            Eigen::Vector2f tex = (t.texs[0] * pw0 + t.texs[1] * pw1 + t.texs[2] * pw2) / invW;

            float u = std::clamp(tex.x(), 0.0f, 1.0f);
            float v = std::clamp(tex.y(), 0.0f, 1.0f);

            int tx = static_cast<int>(u * (texWidth - 1));
            int ty = static_cast<int>((1.0f - v) * (texHeight - 1));

            Color texCol{ 255, 255, 255, 255 };
            if (!albedoTexture.empty())
                texCol = getPixel(albedoTexture, tx, ty, texWidth, texHeight);

            Eigen::Vector3f albedo(
                texCol.r / 255.0f,
                texCol.g / 255.0f,
                texCol.b / 255.0f
            );

            Eigen::Vector3f worldP = t.verts[0] * b0 + t.verts[1] * b1 + t.verts[2] * b2;
            Eigen::Vector3f norm = (t.norms[0] * b0 + t.norms[1] * b1 + t.norms[2] * b2).normalized();

            Eigen::Vector3f color = Eigen::Vector3f::Zero();

            for (const auto& light : lights) {
                Eigen::Vector3f Li = light->getIntensityAt(worldP);

                if (light->getType() == Light::Type::AMBIENT) {
                    color += coeffWiseMultiply(Li, albedo);
                    continue;
                }

                Eigen::Vector3f lightDir = (-light->getDirection(worldP)).normalized();
                Eigen::Vector3f viewDir = (cameraPos - worldP).normalized();

                float diffuse = std::max(0.0f, norm.dot(lightDir));
                Eigen::Vector3f reflectDir = (2.0f * norm * norm.dot(lightDir) - lightDir).normalized();
                float specular = std::pow(std::max(viewDir.dot(reflectDir), 0.0f), 32.0f);

                Eigen::Vector3f diffuseCol = coeffWiseMultiply(Li, albedo) * diffuse;
                Eigen::Vector3f specularCol = Li * specular * 0.5f;

                color += diffuseCol + specularCol;
            }

            const float invGamma = 1.0f / 2.2f;
            auto toByte = [&](float c) -> uint8_t {
                float g = std::pow(std::clamp(c, 0.0f, 1.0f), invGamma);
                return static_cast<uint8_t>(std::lround(g * 255.0f));
            };

            Color out;
            out.r = toByte(color.x());
            out.g = toByte(color.y());
            out.b = toByte(color.z());
            out.a = 255;

            setPixel(image, x, y, width, height, out);
        }
    }
}

void drawMesh(
    std::vector<uint8_t>& image,
    std::vector<float>& zBuffer,
    const Mesh& mesh,
    const std::vector<uint8_t>& tex,
    int texW, int texH,
    const Eigen::Matrix4f& model,
    const Eigen::Matrix4f& worldToClip,
    const std::vector<std::unique_ptr<Light>>& lights,
    int width, int height,
    const Eigen::Vector3f& cameraPos)
{
    for (size_t i = 0; i < mesh.vFaces.size(); ++i) {
        Triangle t;

        const Eigen::Vector3f v0 = mesh.verts[mesh.vFaces[i][0]];
        const Eigen::Vector3f v1 = mesh.verts[mesh.vFaces[i][1]];
        const Eigen::Vector3f v2 = mesh.verts[mesh.vFaces[i][2]];

        const Eigen::Vector3f n0 = mesh.norms[mesh.nFaces[i][0]];
        const Eigen::Vector3f n1 = mesh.norms[mesh.nFaces[i][1]];
        const Eigen::Vector3f n2 = mesh.norms[mesh.nFaces[i][2]];

        Eigen::Vector4f hv0(v0.x(), v0.y(), v0.z(), 1.0f);
        Eigen::Vector4f hv1(v1.x(), v1.y(), v1.z(), 1.0f);
        Eigen::Vector4f hv2(v2.x(), v2.y(), v2.z(), 1.0f);

        Eigen::Vector4f wv0 = model * hv0;
        Eigen::Vector4f wv1 = model * hv1;
        Eigen::Vector4f wv2 = model * hv2;

        Eigen::Vector4f c0 = worldToClip * wv0;
        Eigen::Vector4f c1 = worldToClip * wv1;
        Eigen::Vector4f c2 = worldToClip * wv2;

        if (c0.w() <= 0.0f || c1.w() <= 0.0f || c2.w() <= 0.0f) continue;

        Eigen::Vector4f n0c = c0 / c0.w();
        Eigen::Vector4f n1c = c1 / c1.w();
        Eigen::Vector4f n2c = c2 / c2.w();

        auto sx = [&](float x) { return (x * 0.5f + 0.5f) * (width - 1); };
        auto sy = [&](float y) { return (1.0f - (y * 0.5f + 0.5f)) * (height - 1); };

        t.screen[0] = { sx(n0c.x()), sy(n0c.y()), n0c.z() };
        t.screen[1] = { sx(n1c.x()), sy(n1c.y()), n1c.z() };
        t.screen[2] = { sx(n2c.x()), sy(n2c.y()), n2c.z() };

        t.verts[0] = wv0.head<3>();
        t.verts[1] = wv1.head<3>();
        t.verts[2] = wv2.head<3>();

        Eigen::Matrix3f N = model.block<3, 3>(0, 0).inverse().transpose();

        t.norms[0] = (N * n0).normalized();
        t.norms[1] = (N * n1).normalized();
        t.norms[2] = (N * n2).normalized();

        t.texs[0] = mesh.texs[mesh.tFaces[i][0]];
        t.texs[1] = mesh.texs[mesh.tFaces[i][1]];
        t.texs[2] = mesh.texs[mesh.tFaces[i][2]];

        t.clipW = { c0.w(), c1.w(), c2.w() };
        t.ndc = { n0c, n1c, n2c };

        drawTriangle(image, width, height, zBuffer, t, lights, tex, texW, texH, cameraPos);
    }
}

int main()
{
    const int width = 512, height = 512;
    const int channels = 4;

    std::vector<uint8_t> image(width * height * channels);
    std::vector<float> zBuffer(width * height, 1.0f);

    Eigen::Matrix4f projection = projectionMatrix(height, width);

    Eigen::Vector3f cam(-7.0f, 0.0f, -9.0f);
    Eigen::Matrix4f camToWorld = translationMatrix(cam) * rotateYMatrix(-1.5f);

    Eigen::Matrix4f worldToClip = projection * camToWorld.inverse();

    std::vector<std::unique_ptr<Light>> lights;
    lights.emplace_back(new AmbientLight(Eigen::Vector3f(0.30f, 0.12f, 0.06f)));
    lights.emplace_back(new DirectionalLight(Eigen::Vector3f(0.4f, 0.4f, 0.4f), Eigen::Vector3f(0.0f, 3.0f, 0.0f)));
    lights.emplace_back(new PointLight(Eigen::Vector3f(6.00f, 2.50f, 1.20f), Eigen::Vector3f(0.0f, 2.0f, -11.0f)));

    struct SceneObject { std::string name; std::string texture; Eigen::Vector3f position; float rotY; };

    std::vector<SceneObject> objects = {
        {"Arches", "mur-bas_diffuse", {0,0,0}, 0.0f},
        {"ArchesInner", "arche.001_diffuse", {0,0,0}, 0.0f},
        {"ArchesUpper", "mur-haut_diffuse", {0,0,0}, 0.0f},
        {"TableLegs", "chain_diffuse", {0,0,0}, 0.0f},
        {"OuterRoof", "arche_diffuse", {0,0,0}, 0.0f},
        {"OuterPillars", "colonne-ext_diffuse", {0,0,0}, 0.0f},
        {"HalfPillars", "colonne-ext_diffuse", {0,0,0}, 0.0f},
        {"WindowFrame", "mur-bas-ext_diffuse", {0,0,0}, 0.0f},
        {"Flooring", "material_24_diffuse", {0,0,0}, 0.0f},
        {"RoofDetail", "arche.002_diffuse", {0,0,0}, 0.0f},
        {"WallDetail", "debord-mur_diffuse", {0,0,0}, 0.0f},
        {"PillarBase", "pied-colonne_diffuse", {0,0,0}, 0.0f},
        {"InnerPillar_one", "colonne_diffuse", {0,0,0}, 0.0f},
        {"InnerPillar_two", "colonne_diffuse", {0,0,0}, 0.0f},
        {"AltarBase", "estrade-contour_specularGlossiness", {0,0,0}, 0.0f},
        {"AltarBaseDetail", "estrade-contour_specularGlossiness", {0,0,0}, 0.0f},
        {"AltarTable", "autel_specularGlossiness", {0,0,0}, 0.0f},
        {"WindowDetailed", "vitrail-fond.005_diffuse", {0,0,0}, 0.0f},
        {"WindowBasic", "vitrail-bas_diffuse", {0,0,0}, 0.0f}
    };

    for (auto& o : objects) {
        Mesh mesh = loadMeshFile(std::string("../models/") + o.name + std::string(".obj"));

        std::vector<uint8_t> tex;
        unsigned tw = 1, th = 1;
        
        
        lodepng::decode(tex, tw, th, std::string("../models/") + o.texture + std::string(".png"));

        Eigen::Matrix4f model = translationMatrix(o.position) * rotateYMatrix(o.rotY);

        drawMesh(image, zBuffer, mesh, tex, static_cast<int>(tw), static_cast<int>(th), model, worldToClip, lights, width, height, cam);
    }

    lodepng::encode("output.png", image, width, height);
    saveZBufferImage("zBuffer.png", zBuffer, width, height);

    return 0;
}