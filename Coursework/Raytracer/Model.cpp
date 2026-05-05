#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "Model.hpp"

Model::Model(const char *filename) : verts_(), faces_(), vts_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail()) throw std::runtime_error("Couldn't open input model file!");
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {       // read 2 characters and check the line starts with "v"
            iss >> trash;
            Eigen::Vector3f v;
            for (int i=0;i<3;i++) iss >> v[i];
            verts_.push_back(v);
        }
        else if (!line.compare(0, 3, "vt ")) { // read 3 characters and check the line starts with "vt "
            iss >> trash;
            iss >> trash;
            Eigen::Vector2f vt;
            for (int i=0; i<2; i++) iss >> vt[i]; 
            vts_.push_back(vt);
        }
        if (!line.compare(0, 3, "vn ")) {       // read 2 characters and check the line starts with "v"
            iss >> trash;
            iss >> trash;
            Eigen::Vector3f vn;
            for (int i=0;i<3;i++) iss >> vn[i];
            vns_.push_back(vn);
        }
        else if (!line.compare(0, 2, "f ")) { // f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ... making assumption v1==vt1 etc.
            std::vector<VertexIndices> f;
            VertexIndices idx;
            int itrash;
            iss >> trash;
            while (iss >> idx.vert >> trash >> idx.tex >> trash >> idx.norm) { // read in v_i to idx and discard /vt_i/vn_i
                idx.vert--; // in wavefront obj all indices start at 1, not zero, we need them to start at zero
                idx.tex--;
                idx.norm--;

                f.push_back(idx);
            }
            faces_.push_back(f);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
    std::cerr << "Model loaded: " << filename << "\n";
    std::cerr << "verts: " << verts_.size()
        << " faces: " << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() const {
    return (int)verts_.size();
}

int Model::nfaces() const {
    return (int)faces_.size();
}

bool Model::hasNormals() const {
    return vns_.size() > 0;
}

std::vector<VertexIndices> Model::face(int idx) const {
    return faces_[idx];
}

Eigen::Vector3f Model::vert(int i) const {
    return verts_[i];
}

Eigen::Vector2f Model::texCoord(int i) const {
    return vts_[i];
}

Eigen::Vector3f Model::normal(int i) const {
    return vns_[i];
}

