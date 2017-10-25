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

#include <fstream>
#include <iostream>
#include <string.h>
#include <stack>
#include "animation.h"

void PrintJointRecursive(SkeletonJoint* joint, int depth)
{
	string out = "";
	for (int i = 0; i < depth; i++)
		out += "_";
	cout << out + joint->GetName() + "\n";

	for (auto child : joint->GetDirectChildren())
		PrintJointRecursive(child, depth + 1);
}

void SkeletonJoint::PrintJoint()
{
	PrintJointRecursive(this, 0);
}

Transform::Transform()
{
	m_transformMatrix = mat4(1);
}

Transform::Transform(Transform &transform)
{
	*this = transform;
}

Transform::Transform(mat3 rotation, vec3 origin)
{
	m_transformMatrix = mat4
	(
		vec4(rotation[0][0], rotation[0][1], rotation[0][2], 0),
		vec4(rotation[1][0], rotation[1][1], rotation[1][2], 0),
		vec4(rotation[2][0], rotation[2][1], rotation[2][2], 0),
		vec4(origin.x, origin.y, origin.z, 1)
	);
}

Transform::~Transform() {}

Transform Transform::GetInverse()
{
	Transform t;
	
}

vec3 Transform::GetOrigin()
{
	return m_transformMatrix * vec4(0, 0, 0, 1);
}

void Transform::SetOrigin(vec3 origin)
{
	m_transformMatrix = mat4
	(
		vec4(m_transformMatrix[0][0], m_transformMatrix[0][1], m_transformMatrix[0][2], 0),
		vec4(m_transformMatrix[1][0], m_transformMatrix[1][1], m_transformMatrix[1][2], 0),
		vec4(m_transformMatrix[2][0], m_transformMatrix[2][1], m_transformMatrix[2][2], 0),
		vec4(origin.x, origin.y, origin.z, 1)
	);
}

void Transform::SetRotation(mat3 rotation)
{
	m_transformMatrix = mat4
	(
		vec4(rotation[0][0], rotation[0][1], rotation[0][2], 0),
		vec4(rotation[1][0], rotation[1][1], rotation[1][2], 0),
		vec4(rotation[2][0], rotation[2][1], rotation[2][2], 0),
		vec4(m_transformMatrix[3][0], m_transformMatrix[3][1], m_transformMatrix[3][2], 1)
	);
}

mat3 Transform::GetRotation()
{
	mat3 rotation
	(
		vec3(m_transformMatrix[0][0], m_transformMatrix[0][1], m_transformMatrix[0][2]),
		vec3(m_transformMatrix[1][0], m_transformMatrix[1][1], m_transformMatrix[1][2]),
		vec3(m_transformMatrix[2][0], m_transformMatrix[2][1], m_transformMatrix[2][2])
	);

	return rotation;
}

Transform Transform::GetInverse()
{
	mat3 inverseRotation
	(
		vec3(m_transformMatrix[0][0], m_transformMatrix[1][0], m_transformMatrix[2][0]),
		vec3(m_transformMatrix[0][1], m_transformMatrix[1][1], m_transformMatrix[2][1]),
		vec3(m_transformMatrix[0][2], m_transformMatrix[1][2], m_transformMatrix[2][2])
	);

	vec3 origin(m_transformMatrix[3][0], m_transformMatrix[3][1], m_transformMatrix[3][2]);

	return Transform(inverseRotation, -origin);
}

void QuerySkeletalAnimationRecursive
(
/*Defines the recursion parameters */
SkeletonJoint* joint,
Transform& cumulativeTransform,
unordered_map<string, vector<Transform>>& jointTransforms,
float skeletonScale,
/*Defines the query inputs*/
int frameIndex,
int skeletonIndex,
/*Defines the query outputs*/
vector<vec3>* jointPositions,
unordered_map<string, vec3>* jointPositionsByName,
vector<pair<vec3, vec3>>* segmentPositions,
unordered_map<string, Transform>* cumulativeTransformsByName
)
{
	vec3 jointPositionL = joint->GetLocalOffset();
	vec3 jointPositionW = cumulativeTransform * vec4(jointPositionL[0], jointPositionL[1], jointPositionL[2], 1) * skeletonScale;

	if (cumulativeTransformsByName)
	{
		if (cumulativeTransformsByName->find(joint->GetName()) == cumulativeTransformsByName->end())
			cumulativeTransformsByName->emplace(joint->GetName(), cumulativeTransform);
	}

	if (jointPositions)
		jointPositions->push_back(jointPositionW);

	if (jointPositionsByName)
	{
		if (jointPositionsByName->find(joint->GetName()) == jointPositionsByName->end())
			jointPositionsByName->emplace(joint->GetName(), jointPositionW);
	}

	Transform nextCumulativeTransform;
	if (joint->GetDirectChildren().size()) // Leaf joints do not have transforms, let's not try looking for them
		nextCumulativeTransform = cumulativeTransform * jointTransforms[joint->GetName()][frameIndex];

	for (auto child : joint->GetDirectChildren())
	{
		if (segmentPositions)
		{
			vec3 childPositionL = child->GetLocalOffset();
			vec3 childPositionW = nextCumulativeTransform * vec4(childPositionL[0], childPositionL[1], childPositionL[2], 1) * skeletonScale;

			segmentPositions->push_back(pair<vec3, vec3>(nextCumulativeTransform.GetOrigin() * skeletonScale, childPositionW));
		}

		QuerySkeletalAnimationRecursive(
			child,
			nextCumulativeTransform,
			jointTransforms,
			skeletonScale,
			frameIndex,
			skeletonIndex,
			jointPositions,
			jointPositionsByName,
			segmentPositions,
			cumulativeTransformsByName);
		
	}
}

void SkeletalMotion::QuerySkeletalAnimation
(
/*Defines the query inputs*/
int frameIndex,
int skeletonIndex,
bool addRootOffset,
/*Defines the query outputs*/
vector<vec3>* jointPositions,
unordered_map<string, vec3>* jointPositionsByName,
vector<pair<vec3, vec3>>* segmentPositions,
unordered_map<string, Transform>* cumulativeTransformsByName
)
{
	if (!jointPositions && !jointPositionsByName && !segmentPositions && !cumulativeTransformsByName)
		return;

	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	Transform rootTransform = Transform();

	if (addRootOffset)
		rootTransform.SetOrigin(m_rootTrajectories[frameIndex][skeletonIndex]);

	// Build world pose of the skeleton by traversing the skeleton recursively and fetching joint rotation at the right frame.
	// fills out provided containers while traversing

	QuerySkeletalAnimationRecursive
	(
		root, 
		rootTransform, 
		m_jointTransforms,
		m_skeletonScale,
		frameIndex, 
		skeletonIndex,
		jointPositions,
		jointPositionsByName, 
		segmentPositions,
		cumulativeTransformsByName
	);
}

void SkeletonJoint::QuerySkeleton(unordered_map<string, SkeletonJoint*>* jointPointersByNames, vector<pair<string, string>>* bonesByJointNames)
{
	// Traverse skeleton recursively and fill out provided containers.
	if (jointPointersByNames)
	{
		if (jointPointersByNames->find(m_name) == jointPointersByNames->end())
			jointPointersByNames->emplace(m_name, this);
	}

	for (auto child : m_childJoints)
	{
		if (bonesByJointNames)
		{
			bonesByJointNames->push_back(pair<string, string>(m_name, child->GetName()));
		}

		child->QuerySkeleton(jointPointersByNames, bonesByJointNames);
	}
}

void SkeletalMotion::SetNormalizedScaleWithMultiplier(float scaleCoeff)
{
	vector<vec3> jointPositions;
	QuerySkeletalAnimation(0, 0, false, &jointPositions, NULL, NULL, NULL);

	float maxLength = 0;
	for (vec3 position : jointPositions)
	{
		if (maxLength < position.length())
			maxLength = position.length();
	}

	m_skeletonScale = scaleCoeff / maxLength;
}

void SkeletalMotion::SetNormalizedScale()
{
	SetNormalizedScaleWithMultiplier(1.0f);
}

SkeletalMotion::~SkeletalMotion() {}