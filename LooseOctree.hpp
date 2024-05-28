﻿#pragma once

#include <boost/call_traits.hpp>
#include <glm/glm.hpp>
#include <vector>
#include <array>
#include <span>

constexpr uint32_t NONE_INDEX = static_cast<uint32_t>(-1);

struct LooseOctreeElementId
{
	uint32_t nodeIndex;
	uint32_t elementIndex;

	LooseOctreeElementId()
		: nodeIndex(NONE_INDEX)
		, elementIndex(NONE_INDEX)
	{
			
	}
	LooseOctreeElementId(const uint32_t nodeIndex, const uint32_t elementIndex)
		: nodeIndex(nodeIndex)
		, elementIndex(elementIndex)
	{
			
	}
	bool IsValidId() const
	{
		return nodeIndex != NONE_INDEX && elementIndex != NONE_INDEX;
	}
};
struct BoxCenterAndExtent
{
	glm::vec3 center;
	glm::vec3 extent;

	BoxCenterAndExtent(const glm::vec3& center, const glm::vec3& extent)
		: center(center)
		, extent(extent)
	{
			
	}
	BoxCenterAndExtent()
		: center(0)
		, extent(0)
	{
			
	}
	static inline bool Intersect(const BoxCenterAndExtent& A, const BoxCenterAndExtent& B)
	{
		const glm::vec3 centerDifference = glm::abs(A.center - B.center);
		const glm::vec3 compositeExtent = A.extent + B.extent;
		
		return !glm::any(glm::greaterThan(centerDifference, compositeExtent));
	}
};

template<typename TElement, typename TSemantics>
class LooseOctree
{
	static_assert(TSemantics::MaxDepthCount >= 1, "MaxDepthCount must be greater than or equal to 1");
	static_assert(TSemantics::MaxElementsPerLeaf >= 1, "MaxElementsPerLeaf must be greater than or equal to 1");
	static_assert(TSemantics::MinInclusiveElementsPerNode >= 1, "MinInclusiveElementsPerNode must be greater than or equal to 1");
	static_assert(TSemantics::MaxElementsPerLeaf >= TSemantics::MinInclusiveElementsPerNode, "MaxElementsPerLeaf must be greater than or equal to MinInclusiveElementsPerNode");
private:
	using NodeIndex = uint32_t;
	struct ChildNodeRef
	{
		uint8_t childNodeIndex;

		ChildNodeRef(const uint8_t childNodeIndex)
			: childNodeIndex(childNodeIndex)
		{
			
		}
		
		ChildNodeRef(const glm::bvec3& para)
			: childNodeIndex(((para.x ? 1u : 0u) << 0) | ((para.y ? 1u : 0u) << 1) | ((para.z ? 1u : 0u) << 2))
		{
			
		}

		inline bool IsNull() const
		{
			return childNodeIndex >= 8;
		}
	};
	
	class ChildNodeSubset
	{
	public:

		union
		{
			struct 
			{
				uint32_t positiveX : 1;
				uint32_t positiveY : 1;
				uint32_t positiveZ : 1;
				uint32_t negativeX : 1;
				uint32_t negativeY : 1;
				uint32_t negativeZ : 1;
			};

			struct
			{
				uint32_t positiveChildBits : 3;
				uint32_t negativeChildBits : 3;
			};

			uint32_t childBits : 6;

			uint32_t allBits;
		};

		ChildNodeSubset()
		:	allBits(0)
		{}

		ChildNodeSubset(const ChildNodeRef childRef)
		:	allBits(0)
		{
			positiveChildBits = childRef.childNodeIndex;
			negativeChildBits = ~childRef.childNodeIndex;
		}

		inline bool Contains(const ChildNodeRef childRef) const
		{
			const ChildNodeSubset childSubset(childRef);
			return (childBits & childSubset.childBits) == childSubset.childBits;
		}
	};
	
	struct NodeContext
	{
		BoxCenterAndExtent bounds;
		uint32_t level;

		NodeContext(const BoxCenterAndExtent& bounds, const uint32_t level = 0)
			: bounds(bounds)
			, level(level)
		{
			
		}
	};
	struct Node
	{
		NodeIndex childNodeStartIndex;
		uint32_t inclusiveElementCount;

		bool IsLeaf() const
		{
			return childNodeStartIndex == NONE_INDEX;
		}

		Node()
			: childNodeStartIndex(NONE_INDEX)
			, inclusiveElementCount(0)
		{
			
		}
	};
	struct OffsetAndExtent
	{
		float offset;
		float extent;
		
		OffsetAndExtent(const float offset, const float extent)
			: offset(offset)
			, extent(extent)
		{
			
		}
		OffsetAndExtent()
			: offset(0)
			, extent(0)
		{
			
		}
	};
	
	NodeContext rootNodeContext;
	std::vector<Node> treeNodes;
	// Indexed by compacted indexs
	std::vector<NodeIndex> parentNodeIndexs;
	std::vector<std::vector<TElement>> elementVectors;
	// Compacted indexs
	std::vector<NodeIndex> freeNodeStartIndexs;
	std::array<OffsetAndExtent, TSemantics::MaxDepthCount> levelOffsetAndExtents;

	static inline NodeIndex ToCompactNodeIndex(const NodeIndex nodeIndex)
	{
		// 1 for the existed root
		return (nodeIndex - 1) / 8;
	}

	static inline NodeIndex FromCompactNodeIndex(const NodeIndex compactNodeIndex)
	{
		// 1 for the existed root
		return compactNodeIndex * 8 + 1;
	}

	NodeIndex AllocateEightNodes()
	{
		NodeIndex nodeStartIndex = NONE_INDEX;
		if (freeNodeStartIndexs.empty())
		{
			nodeStartIndex = static_cast<uint32_t>(treeNodes.size());

			treeNodes.insert(treeNodes.end(), 8, Node());
			parentNodeIndexs.emplace_back(NONE_INDEX);
			elementVectors.insert(elementVectors.end(), 8, {});
		}
		else
		{
			nodeStartIndex = FromCompactNodeIndex(freeNodeStartIndexs.back());

			freeNodeStartIndexs.pop_back();
		}
		return nodeStartIndex;
	}

	void FreeEightNodes(const NodeIndex nodeStartIndex)
	{
		std::fill_n(treeNodes.begin() + nodeStartIndex, 8, Node());
		parentNodeIndexs[ToCompactNodeIndex(nodeStartIndex)] = NONE_INDEX;
		freeNodeStartIndexs.emplace_back(ToCompactNodeIndex(nodeStartIndex));
	}

	inline ChildNodeRef GetContainingChildNodeRef(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const;
	inline NodeContext GetChildNodeContext(const NodeContext& nodeContext, const ChildNodeRef childNodeRef) const;
	inline ChildNodeSubset GetIntersectingChildNodeSubset(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const;

	static inline std::array<OffsetAndExtent, TSemantics::MaxDepthCount> BuildOffsetAndExtents(const float extent);
public:
	LooseOctree(const glm::vec3& center, const float extent)
		: rootNodeContext(BoxCenterAndExtent(center, glm::vec3(extent)))
		, treeNodes({ Node() })
		, parentNodeIndexs()
		, elementVectors({ {} })
		, freeNodeStartIndexs()
		, levelOffsetAndExtents(BuildOffsetAndExtents(extent))
	{
		
	}

private:
 	void AddElementInternal(NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& newElementBounds, typename boost::call_traits<TElement>::const_reference newElement);
	void CollapseNodesInternal(const NodeIndex curNodeIndex, std::vector<TElement>& collapsedNodeElements);
	void CollapseNodesInternal(const NodeIndex curNodeIndex);
	template<typename TIterateFunc>
	void FindElementsWithBoundsTestInternal(const NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& testBounds, const TIterateFunc& func) const;

public:
	inline void AddElement(typename boost::call_traits<TElement>::const_reference newElement)
	{
		AddElementInternal(0, rootNodeContext, TSemantics::GetBoundingBox(newElement), newElement);
	}

	void RemoveElement(const LooseOctreeElementId elementId)
	{
		// Remove
		{
			auto& curElementVector = elementVectors[elementId.nodeIndex];
			
			TSemantics::SetElementId(curElementVector[elementId.elementIndex], LooseOctreeElementId());
			std::swap(curElementVector[elementId.elementIndex], curElementVector.back());
			curElementVector.pop_back();

			if (elementId.elementIndex < curElementVector.size())
			{
				TSemantics::SetElementId(curElementVector[elementId.elementIndex], elementId);
			}
		}

		NodeIndex collapseNodeIndex = NONE_INDEX;
		// Update the count from bottom to top and find the node index that need to collapse.
		{
			NodeIndex nodeIndex = elementId.nodeIndex;
			while (true)
			{
				auto& curTreeNode = treeNodes[nodeIndex];
				
				--curTreeNode.inclusiveElementCount;
				if (curTreeNode.inclusiveElementCount < TSemantics::MinInclusiveElementsPerNode)
				{
					collapseNodeIndex = nodeIndex;
				}

				if (nodeIndex == 0)
				{
					break;
				}

				nodeIndex = parentNodeIndexs[ToCompactNodeIndex(nodeIndex)];			
			}
		}

		// Can be collapsed and is not a leaf node (retains leaf nodes).
		if (collapseNodeIndex != NONE_INDEX && !treeNodes[collapseNodeIndex].IsLeaf())
		{
			auto& collapseElementVector = elementVectors[collapseNodeIndex];
			auto& collapseTreeNode = treeNodes[collapseNodeIndex];

			// Child nodes have elements, so elements need be re-insert.
			if (collapseElementVector.size() < collapseTreeNode.inclusiveElementCount)
			{
				const uint32_t transferedElementIndex = static_cast<uint32_t>(collapseElementVector.size());
				
				collapseElementVector.reserve(collapseTreeNode.inclusiveElementCount);

				// Collapse eight child nodes.
				{
					const NodeIndex childStartIndex = collapseTreeNode.childNodeStartIndex;
					for (uint8_t childIndex = 0; childIndex < 8; ++childIndex)
					{
						CollapseNodesInternal(childStartIndex + childIndex, collapseElementVector);
					}
				
					collapseTreeNode.childNodeStartIndex = NONE_INDEX;

					FreeEightNodes(childStartIndex);
				}

				for (uint32_t elementIndex = transferedElementIndex; elementIndex < collapseTreeNode.inclusiveElementCount; ++elementIndex)
				{
					TSemantics::SetElementId(collapseElementVector[elementIndex], LooseOctreeElementId(collapseNodeIndex, elementIndex));
				}
			}
			// Child nodes are empty, so just collapse them.
			else if(collapseElementVector.size() == collapseTreeNode.inclusiveElementCount)
			{
				CollapseNodesInternal(collapseNodeIndex);
			}
		}
	}

	template<typename IterateAllElementsFunc>
	inline void FindAllElements(const IterateAllElementsFunc& func) const
	{
		for (const auto& elementVector: elementVectors)
		{
			for (typename boost::call_traits<TElement>::const_reference element: elementVector)
			{
				func(element);
			}
		}
	}
	
	template<typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const BoxCenterAndExtent& testBounds, const IterateBoundsFunc& func) const
	{
		FindElementsWithBoundsTestInternal(0, rootNodeContext, testBounds, func);
	}

	inline void Clear()
	{
		treeNodes.clear();
		elementVectors.clear();
		freeNodeStartIndexs.clear();
		parentNodeIndexs.clear();
		treeNodes.emplace_back(Node());
		elementVectors.emplace_back({});
	}
	
	inline TElement& GetElementById(const LooseOctreeElementId elementId)
	{
		return elementVectors[elementId.nodeIndex][elementId.elementIndex];
	}
	
	inline const TElement& GetElementById(const LooseOctreeElementId elementId) const
	{
		return elementVectors[elementId.nodeIndex][elementId.elementIndex];
	}

	inline bool IsValidElementId(const LooseOctreeElementId elementId) const
	{
		return elementId.IsValidId() && elementId.elementIndex < elementVectors[elementId.nodeIndex].size();
	}

	inline std::span<const TElement> GetElementsForNode(const NodeIndex nodeIndex) const
	{
		return elementVectors[nodeIndex];
	}

	inline OffsetAndExtent GetNodeLevelOffsetAndExtent(const uint32_t level) const
	{
		return levelOffsetAndExtents[level];
	}

	inline const BoxCenterAndExtent& GetRootBounds() const
	{
		return rootNodeContext.bounds;
	}
};

template <typename TElement, typename TSemantics>
typename LooseOctree<TElement, TSemantics>::ChildNodeRef LooseOctree<TElement, TSemantics>::GetContainingChildNodeRef(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];
	
	const auto negativeCenterDifference = queryBounds.center - (nodeContext.bounds.center - childOffsetAndExtent.offset);
	const auto positiveCenterDifference = (nodeContext.bounds.center + childOffsetAndExtent.offset) - queryBounds.center;
	
	const auto minDifference = glm::min(negativeCenterDifference, positiveCenterDifference);
	if(glm::any(glm::greaterThan(queryBounds.extent + minDifference, glm::vec3(childOffsetAndExtent.extent))))
	{
		return ChildNodeRef(8);
	}
	else
	{
		return ChildNodeRef(glm::greaterThan(queryBounds.center, nodeContext.bounds.center));
	}
}

template <typename TElement, typename TSemantics>
typename LooseOctree<TElement, TSemantics>::NodeContext LooseOctree<TElement, TSemantics>::GetChildNodeContext(const NodeContext& nodeContext, const ChildNodeRef childNodeRef) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];

	constexpr auto mask =  glm::uvec3(1u, 2u, 4u);
	const auto flags = glm::equal(mask, glm::uvec3(childNodeRef.childNodeIndex) & mask);
	const auto childNodeCenterOffset = glm::mix(glm::vec3(-childOffsetAndExtent.offset), glm::vec3(+childOffsetAndExtent.offset), flags);

	return NodeContext(
		BoxCenterAndExtent(nodeContext.bounds.center + childNodeCenterOffset, glm::vec3(childOffsetAndExtent.extent)),
		nodeContext.level + 1
	);
}

template <typename TElement, typename TSemantics>
typename LooseOctree<TElement, TSemantics>::ChildNodeSubset LooseOctree<TElement, TSemantics>::GetIntersectingChildNodeSubset(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const
{
	// Load the query bounding box values as VectorRegisters.
	const glm::vec3 queryBoundsMax = queryBounds.center + queryBounds.extent;
	const glm::vec3 queryBoundsMin = queryBounds.center - queryBounds.extent;
	
	const auto& childOffsetAndExtent = this->levelOffsetAndExtents[nodeContext.level + 1];
	// Compute the bounds of the node's children.
	const glm::vec3 positiveChildBoundsMin = nodeContext.bounds.center + glm::vec3(childOffsetAndExtent.offset) - glm::vec3(childOffsetAndExtent.extent);
	const glm::vec3 negativeChildBoundsMax = nodeContext.bounds.center - glm::vec3(childOffsetAndExtent.offset) + glm::vec3(childOffsetAndExtent.extent);
	
	// Intersect the query bounds with the node's children's bounds.
	const auto positiveChildBitsResult = glm::greaterThan(queryBoundsMax, positiveChildBoundsMin);
	const auto negativeChildBitsResult = glm::lessThanEqual(queryBoundsMin, negativeChildBoundsMax);
	
	ChildNodeSubset result{};
	result.positiveX = positiveChildBitsResult.x ? 1: 0;
	result.positiveY = positiveChildBitsResult.y ? 1: 0;
	result.positiveZ = positiveChildBitsResult.z ? 1: 0;
	result.negativeX = negativeChildBitsResult.x ? 1: 0;
	result.negativeY = negativeChildBitsResult.y ? 1: 0;
	result.negativeZ = negativeChildBitsResult.z ? 1: 0;
	return result;
}

template <typename TElement, typename TSemantics>
std::array<typename LooseOctree<TElement, TSemantics>::OffsetAndExtent, TSemantics::MaxDepthCount> LooseOctree<TElement, TSemantics>::BuildOffsetAndExtents(const float extent)
{
	float parentExtent = extent;
		
	std::array<OffsetAndExtent, TSemantics::MaxDepthCount> offsetAndExtents{ };
	offsetAndExtents[0] = OffsetAndExtent(0, extent);
	for(uint32_t depthIndex = 1; depthIndex < TSemantics::MaxDepthCount; ++depthIndex)
	{
		const float tightChildExtent = parentExtent * 0.5f;
		const float looseChildExtent = tightChildExtent * (1.0f + TSemantics::LoosenessRatio);

		offsetAndExtents[depthIndex] = OffsetAndExtent(parentExtent - looseChildExtent, looseChildExtent);

		parentExtent = looseChildExtent;
	}

	return offsetAndExtents;
}

template <typename TElement, typename TSemantics>
void LooseOctree<TElement, TSemantics>::AddElementInternal(NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& newElementBounds, typename boost::call_traits<TElement>::const_reference newElement)
{
 	// treeNodes may be chanaged, so we must index it every time.
	++treeNodes[curNodeIndex].inclusiveElementCount;
 	// Current node is a leaf node, it can be branched or just inserted.
	if (treeNodes[curNodeIndex].IsLeaf())
	{
		// Current node is full and is not minimum, so this node should b branched.
		if (elementVectors[curNodeIndex].size() == TSemantics::MaxElementsPerLeaf && curNodeContext.level < TSemantics::MaxDepthCount - 1)
		{
			std::vector<TElement> tempElementVector{};
			std::swap(tempElementVector, elementVectors[curNodeIndex]);

			const NodeIndex childNodeStartIndex = AllocateEightNodes();
			
			parentNodeIndexs[ToCompactNodeIndex(childNodeStartIndex)] = curNodeIndex;
			
			treeNodes[curNodeIndex].childNodeStartIndex = childNodeStartIndex;
			treeNodes[curNodeIndex].inclusiveElementCount = 0;

			// Re-insert
			for (typename boost::call_traits<TElement>::const_reference childElement : tempElementVector)
			{
				const BoxCenterAndExtent childElementBox = TSemantics::GetBoundingBox(childElement);
				AddElementInternal(curNodeIndex, curNodeContext, childElementBox, childElement);
			}

			// Insert
			AddElementInternal(curNodeIndex, curNodeContext, newElementBounds, newElement);
		}
		// Current node is not full or is minimum, so we can just insert it.
		else
		{
			const uint32_t newElementIndex = static_cast<uint32_t>(elementVectors[curNodeIndex].size());
			elementVectors[curNodeIndex].emplace_back(newElement);
			
			TSemantics::SetElementId(newElement, LooseOctreeElementId(curNodeIndex, newElementIndex));	
		}
	}
 	// Current node is a usual node, it can be distributed or inserted.
	else
	{
		const ChildNodeRef childNodeRef = GetContainingChildNodeRef(curNodeContext, newElementBounds);
		if (childNodeRef.IsNull())
		{
			const uint32_t newElementIndex = static_cast<uint32_t>(elementVectors[curNodeIndex].size());
			elementVectors[curNodeIndex].emplace_back(newElement);
			
			TSemantics::SetElementId(newElement, LooseOctreeElementId(curNodeIndex, newElementIndex));	
		}
		else
		{
			const NodeIndex childNodeIndex = treeNodes[curNodeIndex].childNodeStartIndex + childNodeRef.childNodeIndex;
			const NodeContext childNodeContext = GetChildNodeContext(curNodeContext, childNodeRef);
			AddElementInternal(childNodeIndex, childNodeContext, newElementBounds, newElement);
		}
	}
}

template <typename TElement, typename TSemantics>
void LooseOctree<TElement, TSemantics>::CollapseNodesInternal(const NodeIndex curNodeIndex, std::vector<TElement>& collapsedNodeElements)
{
	auto& curElementVector = elementVectors[curNodeIndex];
	auto& curTreeNode = treeNodes[curNodeIndex];

	if(!curElementVector.empty())
	{
		collapsedNodeElements.insert(collapsedNodeElements.end(), std::make_move_iterator(curElementVector.begin()), std::make_move_iterator(curElementVector.end()));
		curElementVector.clear();
	}

	if (!curTreeNode.IsLeaf())
	{
		const NodeIndex childStartIndex = curTreeNode.childNodeStartIndex;
		for (uint8_t childIndex = 0; childIndex < 8; ++childIndex)
		{
			CollapseNodesInternal(childStartIndex + childIndex, collapsedNodeElements);
		}
			
		curTreeNode.childNodeStartIndex = NONE_INDEX;

		FreeEightNodes(childStartIndex);
	}
}

template <typename TElement, typename TSemantics>
void LooseOctree<TElement, TSemantics>::CollapseNodesInternal(const NodeIndex curNodeIndex)
{
	auto& curTreeNode = treeNodes[curNodeIndex];

	if (!curTreeNode.IsLeaf())
	{
		const NodeIndex childStartIndex = curTreeNode.childNodeStartIndex;
		for (uint8_t childIndex = 0; childIndex < 8; ++childIndex)
		{
			CollapseNodesInternal(childStartIndex + childIndex);
		}

		// Mark the node as a leaf.
		curTreeNode.childNodeStartIndex = NONE_INDEX;

		FreeEightNodes(childStartIndex);
	}
}

template <typename TElement, typename TSemantics>
template <typename TIterateFunc>
void LooseOctree<TElement, TSemantics>::FindElementsWithBoundsTestInternal(const NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& testBounds, const TIterateFunc& func) const
{
	if (treeNodes[curNodeIndex].inclusiveElementCount > 0)
	{
		// Test all local node element.
		for (typename boost::call_traits<TElement>::const_reference element: elementVectors[curNodeIndex])
		{
			if (BoxCenterAndExtent::Intersect(TSemantics::GetBoundingBox(element), testBounds))
			{
				func(element);
			}
		}

		// Test eight child nodes.
		if (!treeNodes[curNodeIndex].IsLeaf())
		{
			const ChildNodeSubset intersectingChildSubset = GetIntersectingChildNodeSubset(curNodeContext, testBounds);
			const NodeIndex childNodeStartIndex = treeNodes[curNodeIndex].childNodeStartIndex;
			for (uint8_t childNodeIndex = 0; childNodeIndex < 8; ++childNodeIndex)
			{
				if(intersectingChildSubset.Contains(ChildNodeRef(childNodeIndex)))
				{
					FindElementsWithBoundsTestInternal(childNodeStartIndex + childNodeIndex, GetChildNodeContext(curNodeContext, ChildNodeRef(childNodeIndex)), testBounds, func);
				}
			}
		}
	}
}
