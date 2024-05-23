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
		ChildNodeRef(uint8_t inIndex = 0)
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
	inline NodeContext GetChildContext(const NodeContext& nodeContext, const ChildNodeRef childNodeRef) const;
	inline glm::vec3 GetChildOffsetVec(const NodeContext& nodeContext, const uint32_t i) const;

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
 	void AddElementInternal(NodeIndex curNodeIndex, const NodeContext& curNodeContext, const BoxCenterAndExtent& elementBox, typename boost::call_traits<TElement>::const_reference element, std::vector<TElement>& tempElementVector)
	{
 		auto& curTreeNode = treeNodes[curNodeIndex];
 		auto& curElementVector = elementVectors[curNodeIndex];
 		
		++curTreeNode.inclusiveElementCount;
		if (curTreeNode.IsLeaf())
		{
			// is full and is not minest node
			if (curElementVector.size() == TSemantics::MaxElementsPerLeaf && curNodeContext.level < TSemantics::MaxDepthCount - 1)
			{
				tempElementVector = std::move(curElementVector);

				const NodeIndex childNodeStartIndex = AllocateEightNodes();
				
				parentNodeIndexs[ToCompactNodeIndex(childNodeStartIndex)] = curNodeIndex;
				
				curTreeNode.childNodeStartIndex = childNodeStartIndex;
				curTreeNode.inclusiveElementCount = 0;

				for (typename boost::call_traits<TElement>::const_reference childElement : tempElementVector)
				{
					const BoxCenterAndExtent childElementBox = TSemantics::GetBoundingBox(childElement);
					AddElementInternal(curNodeIndex, curNodeContext, childElementBox, childElement, tempElementVector);
				}

				tempElementVector.clear();
				AddElementInternal(curNodeIndex, curNodeContext, elementBox, element, tempElementVector);
			}
			// Can add to this node or this is minest node 
			else
			{
				const uint32_t newElementIndex = static_cast<uint32_t>(curElementVector.size());
				curElementVector.emplace_back(element);
				
				TSemantics::SetElementId(element, ElementId(curNodeIndex, newElementIndex));	
			}
		}
		else
		{
			const ChildNodeRef childNodeRef = GetContainingChild(curNodeContext, elementBox);
			if (childNodeRef.IsNULL())
			{
				const uint32_t newElementIndex = static_cast<uint32_t>(curElementVector.size());
				curElementVector.emplace_back(element);
				TSemantics::SetElementId(element, ElementId(curNodeIndex, newElementIndex));	

			}
			else
			{
				const NodeIndex childNodeIndex = curTreeNode.childNodeStartIndex + childNodeRef.childNodeIndex;
				const NodeContext childNodeContext = GetChildContext(curNodeContext, childNodeRef);
				AddElementInternal(childNodeIndex, childNodeContext, elementBox, element, tempElementVector);
				return;
			}
		}
	}
public:
	inline void AddElement(typename boost::call_traits<TElement>::const_reference newElement)
	{
		std::vector<TElement> tempElementVector;
		const BoxCenterAndExtent newElementBounds = TSemantics::GetBoundingBox(newElement);
		AddElementInternal(0, rootNodeContext, newElementBounds, newElement, tempElementVector);
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
typename LooseOctree<TElement, TSemantics>::NodeContext LooseOctree<TElement, TSemantics>::GetChildContext(const NodeContext& nodeContext, const ChildNodeRef childNodeRef) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];

	const auto mask =  glm::uvec3(1u, 2u, 4u);
	const auto flag = glm::equal(mask, glm::uvec3(childNodeRef.childNodeIndex) & mask);
	const auto childNodeCenterOffset = glm::mix(glm::vec3(childOffsetAndExtent.offset), glm::vec3(-childOffsetAndExtent.offset), flag);

	BoxCenterAndExtent childNodeBounds{};
	childNodeBounds.center = nodeContext.bounds.center + childNodeCenterOffset;
	childNodeBounds.extent = glm::vec3(childOffsetAndExtent.extent);
	
	return NodeContext(childNodeBounds, nodeContext.level + 1);
}

template <typename TElement, typename TSemantics>
glm::vec3 LooseOctree<TElement, TSemantics>::GetChildOffsetVec(const NodeContext& nodeContext, const uint32_t i) const
{
	const auto& childOffsetAndExtent = levelOffsetAndExtents[nodeContext.level + 1];
	
	const auto mask =  glm::uvec3(1u, 2u, 4u);
	const auto flag = glm::equal(mask, glm::uvec3(i) & mask);
	const auto childNodeCenterOffset = glm::mix(glm::vec3(childOffsetAndExtent.offset), glm::vec3(-childOffsetAndExtent.offset), flag);
	
	return childNodeCenterOffset;
}
