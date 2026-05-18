#pragma once
#include "Renderable.hpp"
#include "GeomUtil.hpp"
#include "Mesh.hpp"
#include "BVHLeafNode.hpp"
#include <vector>


/// <summary>
/// This is a single node of a Bounding Volume Hierarchy.
/// In this library BVHs are binary trees (each can have at most 2 children.
/// They can be constructed from either a list of renderables or a mesh
/// (provided as a Model instance).
/// Each node splits along a single axis (x, y or z).
/// </summary>
class BVHNode : public Renderable
{
private:
	AABB aabb_;
	int nodeDepth_;
	std::shared_ptr<Renderable> child0_, child1_;
public:

	/// <summary>
	/// Constructs a BVH from a list of Renderable instances.
	/// </summary>
	/// <param name="renderables">The instances to add to the BVH.</param>
	/// <param name="maxDepth">The maximum depth of the binary tree.</param>
	BVHNode(const std::vector<std::shared_ptr<Renderable>>& renderables, int maxDepth)
		:Renderable(nullptr), nodeDepth_(maxDepth)
	{
		// *** YOUR CODE HERE ***

		// 1. Find an AABB surrounding all the renderables, and update the aabb_ member.
		//    Implement this in the getRenderablesAABB() function, at the end of GeomUtil.hpp.
		// 2. Choose an axis to split your AABB along (x, y or z).
		// 3. Make lists of the renderables that fall on either side of the split.
		// 4. For child0_ and child1_, check if we should terminate:
		//     a. If we reach the max depth (i.e. maxDepth == 0)
		//     b. If there are two or fewer Renderables left in the list for this child.
		// 5. If we should terminate, make a new BVHLeafNode and supply the renderables
		//    in this list.
		// 6. If we should not terminate, make a new BVHNode and call this constructor again
		//    recursively. Don't forget to subtract one from maxDepth!
	}

	/// <summary>
	/// This constructor forms a BVH tree from a provided triangle mesh.
	/// At the leaf nodes this uses PartialMesh instances to store the triangle indices
	/// at each node.
	/// Note for BVH accelerated meshes, the modelToWorld transform must be set in this 
	/// constructor.
	/// Internally the BVH is constructed in world space. Vertices are transformed
	/// using the modelToWorld before being used to find AABBs etc.
	/// </summary>
	/// <param name="model">The loaded model instance to construct the mesh BVH from.</param>
	/// <param name="shader">The shader to use when intersecting the mesh.</param>
	/// <param name="maxDepth">Maximum depth of the BVH binary tree.</param>
	/// <param name="modelToWorld">Transform taking the mesh to world space.</param>
	/// <param name="faceIndices">Indices of faces of the mesh. If none are supplied (i.e. this is 
	/// set to nullptr), all faces in the model instance will be used.</param>
	/// <param name="culling">Turn on/off backface culling (same parameter as in the Mesh class).</param>
	BVHNode(const Model& model, const Shader* shader, int maxDepth, const Eigen::Matrix4f &modelToWorld,
		const std::vector<std::vector<VertexIndices>>* faceIndices = nullptr, bool culling=true)
		:Renderable(nullptr)
	{
		std::vector<std::vector<VertexIndices>> myFaceIndices;
		if(faceIndices) 
			getModelAABB(model, *faceIndices, modelToWorld);
		else {
			for (int f = 0; f < model.nfaces(); ++f) {
				std::vector<VertexIndices> face;
				for (int v = 0; v < 3; ++v) {
					face.push_back(model.face(f)[v]);
				}
				myFaceIndices.push_back(face);
			}
			getModelAABB(model, myFaceIndices, modelToWorld);
			faceIndices = &myFaceIndices;
		}

		int splittingAxis = findBestSplittingAxis();
		float splittingLoc = aabb_.centre()[splittingAxis];

		std::vector<std::vector<VertexIndices>> faces0, faces1;

		for (int f = 0; f < faceIndices->size(); ++f) {
			Eigen::Vector3f
				v0 = model.vert((*faceIndices)[f][0].vert),
				v1 = model.vert((*faceIndices)[f][1].vert),
				v2 = model.vert((*faceIndices)[f][2].vert);
			Eigen::Vector3f pos = transformPosition(modelToWorld, (v0 + v1 + v2) / 3.f);
			if (pos[splittingAxis] < splittingLoc) {
				faces0.push_back((*faceIndices)[f]);
			}
			else {
				faces1.push_back((*faceIndices)[f]);
			}
		}

		if (faces0.size() <= 1 || maxDepth <= 0) {
			if (faces0.size() > 0) {
				child0_ = std::make_shared<Mesh>(shader, &model, &faces0, culling);
				child0_->modelToWorld(modelToWorld);
			}
		}
		else {
			child0_ = std::make_shared<BVHNode>(model, shader, maxDepth - 1, modelToWorld, &faces0, culling);
		}
		if (!faces1.empty()) {
			if (faces1.size() <= 1 || maxDepth <= 0) {
				child1_ = std::make_shared<Mesh>(shader, &model, &faces1, culling);
				child1_->modelToWorld(modelToWorld);
			}
			else {
				child1_ = std::make_shared<BVHNode>(model, shader, maxDepth - 1, modelToWorld, &faces1, culling);
			}
		}
	}


	/// <summary>
	/// Finds the best axis to split the BVH along.
	/// This implementation selects the longest axis of the AABB.
	/// </summary>
	int findBestSplittingAxis() const
	{
		int maxExtentAxis = 0;
		float maxExtent = 0.0f;
		for (int axis = 0; axis < 3; ++axis) {
			float extent = aabb_.max[axis] - aabb_.min[axis];
			if (extent > maxExtent) {
				maxExtent = extent;
				maxExtentAxis = axis;
			}
		}

		return maxExtentAxis;
	}

	/// <summary>
	/// Gets an AABB for a triangle mesh, after applying the given modelToWorld transform
	/// That is, the AABB will be in world space.
	/// </summary>
	void getModelAABB(
		const Model& model, const std::vector<std::vector<VertexIndices>>& faceIndices, 
		const Eigen::Matrix4f& modelToWorld)
	{
		for (int i = 0; i < 3; ++i) {
			aabb_.min[i] = std::numeric_limits<float>::max();
			aabb_.max[i] = std::numeric_limits<float>::min();
		}
		for (int f = 0; f < faceIndices.size(); ++f) {
			for (int v = 0; v < 3; ++v) {
				Eigen::Vector3f v0 = model.vert(faceIndices[f][v].vert);
				v0 = transformPosition(modelToWorld, v0);
				for (int i = 0; i < 3; ++i) {
					if (v0[i] < aabb_.min[i]) aabb_.min[i] = v0[i];
					if (v0[i] > aabb_.max[i]) aabb_.max[i] = v0[i];
				}
			}
		}
	}

	virtual AABB getAABB() const override
	{
		return aabb_;
	}

	virtual bool intersect(const Ray& ray, float minT, float maxT, HitInfo& info, IntersectMask mask) const override
	{
		Ray tRay;
		tRay.origin = transformPosition(worldToModel(), ray.origin);
		tRay.direction = transformDirection(worldToModel(), ray.direction);

		// If we don't hit the AABB associated with this node at all, exit early!
		if (!aabb_.intersect(tRay, minT, maxT)) return false;

		// We did hit the AABB, so now we test the children.
		bool hitSomething = false; // Keep track of whether a hit occurred.

		float t = std::numeric_limits<float>::max();
		HitInfo currInfo;
		if (child0_ && child0_->intersect(tRay, minT, maxT, currInfo, mask)) {
			if (currInfo.hitT < t) {
				info = currInfo;
				t = currInfo.hitT;
				hitSomething = true;
			}
		}
		if (child1_ && child1_->intersect(tRay, minT, maxT, currInfo, mask)) {
			if (currInfo.hitT < t) {
				info = currInfo;
				t = currInfo.hitT;
				hitSomething = true;
			}
		}

		return hitSomething;
	}

	/// <summary>
	/// Prints a summary of the entries in this BVH and its children.
	/// The list is indented to reflect the depth of each node in the tree.
	/// </summary>
	virtual std::string print() const override
	{
		std::stringstream indentStr;
		for (int i = 0; i < 10 - nodeDepth_; ++i) {
			indentStr << " ";
		}
		std::string indent = indentStr.str();
		std::stringstream ss;
		ss << indent << "BVH Node depth " << nodeDepth_ << " from\n" << aabb_.min << "to\n" << aabb_.max << "\n"
			<< indent << "Children0:\n";
		ss << indent << child0_->print() << "\n";
		ss << indent << "Children1:\n";
		ss << indent << child1_->print() << "\n";
		return ss.str();
	}

	virtual void modelToWorld(const Eigen::Matrix4f& m) override
	{
		throw(std::runtime_error("Can't transform a BVH node."));
	}
};

