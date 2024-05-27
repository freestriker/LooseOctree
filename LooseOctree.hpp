#pragma once

#include <boost/call_traits.hpp>
#include <glm/glm.hpp>
#include <vector>
#include <array>


struct ElementId
{
	uint32_t nodeIndex;
	uint32_t elementIndex;

	ElementId()
		: nodeIndex(uint32_t(-1))
		, elementIndex(uint32_t(-1))
	{
			
	}
	ElementId(const uint32_t nodeIndex, const uint32_t elementIndex)
		: nodeIndex(nodeIndex)
		, elementIndex(elementIndex)
	{
			
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
		
		return glm::any(glm::greaterThan(centerDifference, compositeExtent));
	}
};

template<typename TElement, typename TSemantics>
class LooseOctree
{
public:
	using NodeIndex = uint32_t;
	static constexpr uint32_t NONE_NODE_INDEX = static_cast<uint32_t>(-1);
private:
	struct ChildNodeRef
	{
		uint8_t childNodeIndex;

		/** Initialization constructor. */
		ChildNodeRef(const uint8_t inX, const uint8_t inY, const uint8_t inZ)
			: childNodeIndex((inX << 0) | (inY << 1) | (inZ << 2))
		{

		}

		/** Initialized the reference with a child index. */
		ChildNodeRef(const uint8_t inIndex = 0)
		:	childNodeIndex(inIndex)
		{

		}

		ChildNodeRef(const glm::bvec3& para)
		:	 childNodeIndex(((para.x ? 1 : 0) << 0) | ((para.y ? 1 : 0) << 1) | ((para.z ? 1 : 0) << 2))
		{
			
		}

		/** Advances the reference to the next child node.  If this was the last node remain, Index will be 8 which represents null. */
		inline void Advance()
		{
			++childNodeIndex;
		}

		/** @return true if the reference isn't set. */
		inline bool IsNULL() const
		{
			return childNodeIndex >= 8;
		}

		inline void SetNULL()
		{
			childNodeIndex = 8;
		}

		inline uint8_t X() const
		{
			return (childNodeIndex >> 0) & 1;
		}

		inline uint8_t Y() const
		{
			return (childNodeIndex >> 1) & 1;
		}

		inline uint8_t Z() const
		{
			return (childNodeIndex >> 2) & 1;
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
				/** Only the bits for the children on the positive side of the splits. */
				uint32_t positiveChildBits : 3;

				/** Only the bits for the children on the negative side of the splits. */
				uint32_t negativeChildBits : 3;
			};

			/** All the bits corresponding to the child bits. */
			uint32_t childBits : 6;

			/** All the bits used to store the subset. */
			uint32_t allBits;
		};

		/** Initializes the subset to be empty. */
		ChildNodeSubset()
		:	allBits(0)
		{}

		/** Initializes the subset to contain a single node. */
		ChildNodeSubset(const ChildNodeRef childRef)
		:	allBits(0)
		{
			// The positive child bits correspond to the child index, and the negative to the NOT of the child index.
			positiveChildBits = childRef.childNodeIndex;
			negativeChildBits = ~childRef.childNodeIndex;
		}

		/** Determines whether the subset contains a specific node. */
		inline bool Contains(const ChildNodeRef childRef) const
		{
			// This subset contains the child if it has all the bits set that are set for the subset containing only the child node.
			const ChildNodeSubset childSubset(childRef);
			return (childBits & childSubset.childBits) == childSubset.childBits;
		}
	};
	
	struct NodeContext
	{
		BoxCenterAndExtent bounds;
		uint32_t level;

		NodeContext(const BoxCenterAndExtent& box, const uint32_t level = 0)
			: bounds(box)
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
			return childNodeStartIndex == NONE_NODE_INDEX;
		}

		Node()
			: childNodeStartIndex(NONE_NODE_INDEX)
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
	const std::array<OffsetAndExtent, TSemantics::MaxDepthCount> levelOffsetAndExtents;

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
		NodeIndex nodeStartIndex = NONE_NODE_INDEX;
		if (freeNodeStartIndexs.empty())
		{
			nodeStartIndex = static_cast<uint32_t>(treeNodes.size());

			treeNodes.insert(treeNodes.end(), 8, Node());
			parentNodeIndexs.emplace_back(NONE_NODE_INDEX);
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
		std::fill_n(treeNodes.begin() + nodeStartIndex, 8, Node(NONE_NODE_INDEX, 0));
		parentNodeIndexs[ToCompactNodeIndex(nodeStartIndex)] = NONE_NODE_INDEX;
		freeNodeStartIndexs.emplace_back(ToCompactNodeIndex(nodeStartIndex));
	}

	inline ChildNodeRef GetContainingChild(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const;
	inline NodeContext GetChildNodeContext(const NodeContext& nodeContext, const ChildNodeRef childNodeRef) const;
	inline glm::vec3 GetChildOffsetVec(const NodeContext& nodeContext, const uint32_t i) const;
	inline ChildNodeSubset GetIntersectingChildNodeSubset(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const;

	static inline std::array<OffsetAndExtent, TSemantics::MaxDepthCount> BuildOffsetAndExtents(const float extent)
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
 	void AddElementInternal(NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& elementBox, typename boost::call_traits<TElement>::const_reference element)
	{
		++treeNodes[curNodeIndex].inclusiveElementCount;
		if (treeNodes[curNodeIndex].IsLeaf())
		{
			// is full and is not minest node
			if (elementVectors[curNodeIndex].size() == TSemantics::MaxElementsPerLeaf && curNodeContext.level < TSemantics::MaxDepthCount - 1)
			{
				auto tempElementVector = std::move(elementVectors[curNodeIndex]);

				const NodeIndex childNodeStartIndex = AllocateEightNodes();
				
				parentNodeIndexs[ToCompactNodeIndex(childNodeStartIndex)] = curNodeIndex;
				
				treeNodes[curNodeIndex].childNodeStartIndex = childNodeStartIndex;
				treeNodes[curNodeIndex].inclusiveElementCount = 0;

				for (typename boost::call_traits<TElement>::const_reference childElement : tempElementVector)
				{
					const BoxCenterAndExtent childElementBox = TSemantics::GetBoundingBox(childElement);
					AddElementInternal(curNodeIndex, curNodeContext, childElementBox, childElement);
				}
				
				AddElementInternal(curNodeIndex, curNodeContext, elementBox, element);
			}
			// Can add to this node or this is minest node 
			else
			{
				const uint32_t newElementIndex = static_cast<uint32_t>(elementVectors[curNodeIndex].size());
				elementVectors[curNodeIndex].emplace_back(element);
				
				TSemantics::SetElementId(element, ElementId(curNodeIndex, newElementIndex));	
			}
		}
		else
		{
			const ChildNodeRef childNodeRef = GetContainingChild(curNodeContext, elementBox);
			if (childNodeRef.IsNULL())
			{
				const uint32_t newElementIndex = static_cast<uint32_t>(elementVectors[curNodeIndex].size());
				elementVectors[curNodeIndex].emplace_back(element);
				TSemantics::SetElementId(element, ElementId(curNodeIndex, newElementIndex));	
			}
			else
			{
				const NodeIndex childNodeIndex = treeNodes[curNodeIndex].childNodeStartIndex + childNodeRef.childNodeIndex;
				const NodeContext childNodeContext = GetChildNodeContext(curNodeContext, childNodeRef);
				AddElementInternal(childNodeIndex, childNodeContext, elementBox, element);
				return;
			}
		}
	}
public:
	inline void AddElement(typename boost::call_traits<TElement>::const_reference newElement)
	{
		const BoxCenterAndExtent newElementBounds = TSemantics::GetBoundingBox(newElement);
		AddElementInternal(0, rootNodeContext, newElementBounds, newElement);
	}
private:
	void CollapseNodesInternal(const NodeIndex curNodeIndex, std::vector<TElement>& collapsedNodeElements)
	{
		auto& curElementVector = elementVectors[curNodeIndex];
		auto& curTreeNode = treeNodes[curNodeIndex];
		
		collapsedNodeElements.insert(collapsedNodeElements.end(), std::make_move_iterator(curElementVector.begin()), std::make_move_iterator(curElementVector.end()));
		curElementVector.clear();

		if (!curTreeNode.IsLeaf())
		{
			const NodeIndex childStartIndex = curTreeNode.childNodeStartIndex;
			for (uint8_t childIndex = 0; childIndex < 8; ++childIndex)
			{
				CollapseNodesInternal(childStartIndex + childIndex, collapsedNodeElements);
			}

			// Mark the node as a leaf.
			curTreeNode.childNodeStartIndex = NONE_NODE_INDEX;

			FreeEightNodes(childStartIndex);
		}
	}
	void CollapseNodesInternal(const NodeIndex curNodeIndex)
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
			curTreeNode.childNodeStartIndex = NONE_NODE_INDEX;

			FreeEightNodes(childStartIndex);
		}
	}
public:
	inline void RemoveElement(const ElementId elementId)
	{
		// Remove
		{
			auto& curElementVector = elementVectors[elementId.nodeIndex];
			
			std::swap(curElementVector[elementId.elementIndex], curElementVector.back());
			curElementVector.pop_back();

			if (elementId.elementIndex < curElementVector.size())
			{
				TSemantics::SetElementId(curElementVector[elementId.elementIndex], elementId);
			}
		}

		NodeIndex collapseNodeIndex = NONE_NODE_INDEX;
		{
			// Update the inclusive element counts for the nodes between the element and the root node,
			// and find the largest node that is small enough to collapse.
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

		// Collapse the largest node that was pushed below the threshold for collapse by the removal.
		if (collapseNodeIndex != NONE_NODE_INDEX && !treeNodes[collapseNodeIndex].IsLeaf())
		{
			auto& collapseElementVector = elementVectors[collapseNodeIndex];
			auto& treeNode = treeNodes[collapseNodeIndex];
			
			if (collapseElementVector.size() < treeNode.inclusiveElementCount)
			{
				std::vector<TElement> tempElementStorage{};
				tempElementStorage.reserve(treeNode.inclusiveElementCount);
				// Gather the elements contained in this node and its children.
				CollapseNodesInternal(collapseNodeIndex, tempElementStorage);
				collapseElementVector = std::move(tempElementStorage);

				for (uint32_t elementIndex = 0; elementIndex < collapseElementVector.size(); ++elementIndex)
				{
					// Update the external element id for the element that's being collapsed.
					TSemantics::SetElementId(collapseElementVector[elementIndex], ElementId(collapseNodeIndex, elementIndex));
				}
			}
			else if(collapseElementVector.size() == treeNode.inclusiveElementCount)
			{
				CollapseNodesInternal(collapseNodeIndex);
			}
		}
	}
public:
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
private:
	template<typename IterateFunc>
	void FindElementsWithBoundsTestInternal(const NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& boxBounds, const IterateFunc& func) const
	{
		if (treeNodes[curNodeIndex].inclusiveElementCount > 0)
		{
			for (typename boost::call_traits<TElement>::const_reference element: elementVectors[curNodeIndex])
			{
				if (BoxCenterAndExtent::Intersect(TSemantics::GetBoundingBox(element), boxBounds))
				{
					func(element);
				}
			}

			if (!treeNodes[curNodeIndex].IsLeaf())
			{
				const ChildNodeSubset intersectingChildSubset = GetIntersectingChildNodeSubset(curNodeContext, boxBounds);
				const NodeIndex childNodeStartIndex = treeNodes[curNodeIndex].childNodeStartIndex;
				for (uint8_t childNodeIndex = 0; childNodeIndex < 8; ++childNodeIndex)
				{
					if(intersectingChildSubset.Contains(ChildNodeRef(childNodeIndex)))
					{
						FindElementsWithBoundsTestInternal(childNodeStartIndex + childNodeIndex, GetChildNodeContext(curNodeContext, ChildNodeRef(childNodeIndex)), boxBounds, func);
					}
				}
			}
		}
	}
public:
	template<typename IterateBoundsFunc>
	inline void FindElementsWithBoundsTest(const BoxCenterAndExtent& boxBounds, const IterateBoundsFunc& func) const
	{
		FindElementsWithBoundsTestInternal(0, rootNodeContext, boxBounds, func);
	}
};

template <typename TElement, typename TSemantics>
typename LooseOctree<TElement, TSemantics>::ChildNodeRef LooseOctree<TElement, TSemantics>::GetContainingChild(const NodeContext& nodeContext, const BoxCenterAndExtent& queryBounds) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];
	
	const auto childOffset = glm::vec3(childOffsetAndExtent.offset);
	const auto negativeCenterDifference = queryBounds.center - (nodeContext.bounds.center - childOffset);
	const auto positiveCenterDifference = (nodeContext.bounds.center + childOffset) - queryBounds.center;
	
	const auto minDifference = glm::min(positiveCenterDifference, negativeCenterDifference);
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
	const auto flag = glm::equal(mask, glm::uvec3(childNodeRef.childNodeIndex) & mask);
	const auto childNodeCenterOffset = glm::mix(glm::vec3(-childOffsetAndExtent.offset), glm::vec3(+childOffsetAndExtent.offset), flag);

	BoxCenterAndExtent childNodeBounds{};
	childNodeBounds.center = nodeContext.bounds.center + childNodeCenterOffset;
	childNodeBounds.extent = glm::vec3(childOffsetAndExtent.extent);
	
	return NodeContext(childNodeBounds, nodeContext.level + 1);
}

template <typename TElement, typename TSemantics>
glm::vec3 LooseOctree<TElement, TSemantics>::GetChildOffsetVec(const NodeContext& nodeContext, const uint32_t i) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];

	constexpr auto mask =  glm::uvec3(1u, 2u, 4u);
	const auto flag = glm::equal(mask, glm::uvec3(i) & mask);
	const auto childNodeCenterOffset = glm::mix(glm::vec3(childOffsetAndExtent.offset), glm::vec3(-childOffsetAndExtent.offset), flag);
	
	return childNodeCenterOffset;
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