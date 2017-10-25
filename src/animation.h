/*
	CBVH++: Loads a skeletal animation
	Copyright(C) 2017 Vincent Petrella

	This program is free software : you can redistribute it and / or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see <https://www.gnu.org/licenses/>.
*/

#include "../include/glm/glm.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>

using namespace std;
using namespace glm;

/*
	Class SkeletonJoint:
	
	This is the tree node that holds information about a specific joint. A skeleton is a tree of these things.
*/

class Transform
{
public:
	Transform();
	Transform(mat3 rotation, vec3 origin);
	~Transform();

	void SetOrigin(vec3 origin);
	void SetRotation(mat3 rotation);

	vec3 GetOrigin();
	mat3 GetRotation();

	Transform GetInverse();

	Transform Transform::operator*(const Transform& rhs) const
	{
		return Transform(this->m_transformMatrix * rhs.m_transformMatrix);
	}

	vec4 Transform::operator*(const vec4& rhs) const
	{
		return m_transformMatrix * rhs;
	}

	vec3 Rotate(vec3 direction) { return vec3(*this*vec4(direction, 0)); }

	vec3 TransformPoint(vec3 point) { return vec3(*this*vec4(point, 1)); }

private:
	Transform(mat4 transformMatrix) { m_transformMatrix = transformMatrix; }
	mat4 m_transformMatrix;
};

class SkeletonJoint
{
public:

	SkeletonJoint(string name, vector<SkeletonJoint*> childJoints, vec3 &localOffset)
	{
		m_name = name;
		m_childJoints = childJoints;
		m_localOffset = localOffset;
	}

	~SkeletonJoint() {};

	string		GetName()			{ return m_name; }
	
	vec3		GetLocalOffset()	{ return m_localOffset; }

	void		ApplyOffsetNormalization(float normalizer)	{ m_localOffset /= normalizer; }

	/*
		Returns a vector of pointers to all the direct descendance of this joint.
		An end joint will return a empty vector.
	*/
	vector<SkeletonJoint*> GetDirectChildren()	{ return m_childJoints; }

	/*
		QuerySkeleton:
		Use to retrieve information from the Skeleton without having to do it recursively yourself.
		Careful there... this has to traverse the skeleton, and may therefore be costly!
	*/
	void QuerySkeleton(unordered_map<string, SkeletonJoint*>* jointPointersByNames, vector<pair<string, string>>* bonesByJointNames);

	void PrintJoint();

private:
	string					m_name;
	vec3					m_localOffset;
	vector<SkeletonJoint*>	m_childJoints;
};

class SkeletalMotion
{
public:

	SkeletalMotion(
		string name,
		vector<vector<vec3>> rootTrajectories,
		unordered_map<string, vector<Transform>> jointTransforms,
		vector<SkeletonJoint*> skeletonRoots,
		float samplingRate,
		int	  frameCount)
	{
		m_name = name;
		m_rootTrajectories = rootTrajectories;
		m_jointTransforms = jointTransforms;
		m_skeletonRoots = skeletonRoots;
		m_samplingRate = samplingRate;
		m_frameCount = frameCount;
		m_skeletonScale = 1.0f;
	};

	~SkeletalMotion();

	string GetName()		{ return m_name; }

	/*
		Returns the sampling rate in secs as specified in the animation file 
	*/

	float GetSamplingRate() { return m_samplingRate; }
	
	/*
		Returns the length of the animation, in number of frame. Length in time = framecout * sampling_rate
	*/
	int GetFrameCount()		{ return m_frameCount; }

	/*
		Returns the Root joint of the desired skeleton defined in the animation clip.
	*/
	SkeletonJoint* GetRoot(int index){ return m_skeletonRoots[index]; }

	/*
		QuerySkeletalAnimation:
		Use this function to retrieve information about the pose of an animation at a frameIndex, in the required formats.
		Careful there... this traverses the skeleton and assembles the full pose. Call only once a frame if possible <3.
	*/
	void QuerySkeletalAnimation
	(
		/*Defines the query inputs*/
		int frameIndex,
		int skeletonIndex,
		bool addRootOffset,
		/*Defines the query outputs*/
		vector<vec3>* jointPositions = NULL,
		unordered_map<string, vec3>* jointPositionsByName = NULL,
		vector<pair<vec3, vec3>>* segmentPositions = NULL,
		unordered_map<string, Transform>* cumulativeTransformsByName = NULL
	);

	/*
		Compute and set a normalizing scale so that differnt skeleton definition appear at the same scale in the application
	*/
	void SetNormalizedScale();

	/*
		Compute and set a normalizing scale, with a multiplier applied (for visualization)
	*/
	void SetNormalizedScaleWithMultiplier(float scaleCoeff);

	/*
		Sets the scale of positions information retrieved.
	*/
	void SetScale(float scale) { m_skeletonScale = scale; }

	/*
		Queries the local transform of a joint using it's name, for a specific frame index.
	*/
	Transform GetLocalTransformByName(std::string name, int frameIndex) 
	{
		Transform tr = m_jointTransforms[name][frameIndex];
		return tr;
	}

private:
	string m_name;
	vector<vector<vec3>>			m_rootTrajectories;
	unordered_map < string,
		vector < Transform >>		m_jointTransforms;

	vector<SkeletonJoint*>			m_skeletonRoots;
	float							m_samplingRate;
	int								m_frameCount;
	float							m_skeletonScale;
public:

	/*
		BVHImport: 
		Creates a SkeletalMotion object on the heap and returns a pointer to it.
		Input a valide file path and things should be alright.
	*/
	static SkeletalMotion* BVHImport(string bvhFilePath);
};