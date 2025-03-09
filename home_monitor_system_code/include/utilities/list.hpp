#pragma once
#include <stdint.h>

template <typename T, int32_t N = 10>
struct List
{
    int32_t head = -1;
    int32_t free = -1;

    struct Node
    {
        T data;
        int32_t nextNode = -1;
    } nodes[N];

    List()
    {
        for (int32_t i = 0; i < N - 1; i++)
        {
            nodes[i].nextNode = i + 1;
        }
        nodes[N - 1].nextNode = -1;
        
        free = 0;
        head = -1;
    }

    /// @brief Adds new element to fron of list
    /// @param data reference to data
    /// @return on success return 0 else -1
    int add(T& data)
    {
        // List is empty
        if(head == -1)
        {
            head = free;
            free = nodes[free].nextNode;
            nodes[head].data = data;
            nodes[head].nextNode = -1;
            return 0;
        }
        
        // List is full!
        if(free == -1)
            return -1; 
        
        int32_t prevHead = head;
        int32_t prevFree = free;
        free = nodes[free].nextNode;
        head = prevFree;
        nodes[head].nextNode = prevHead;
        nodes[head].data = data;       

        return 0;
    }

    int remove(uint32_t index)
    {
        if(index >= N)
            return -1;

        // List is empty
        if(head == -1)
            return -1;

        int32_t prevHead = head;
        int32_t prevFree = free;

        // First element
        if(index == 0)
        {
            head = nodes[head].nextNode;
            free = prevHead;
            nodes[free].nextNode = prevFree;
            return 0;
        }

        int32_t prevToRemove = head;
        for (uint32_t i = 0; i < index - 1; i++)
        {
            // End of list
            if(prevToRemove == -1)
                return -1;            
            prevToRemove = nodes[prevToRemove].nextNode;
        }
        
        int32_t toRemove = nodes[prevToRemove].nextNode;
        nodes[prevToRemove].nextNode = nodes[toRemove].nextNode;
        free = toRemove;
        nodes[free].nextNode = prevFree;

        return 0;
    }

    int valueAt(uint32_t index, T*& ptr)
    {
        // assert(index >= N);
        // if(index >= N)
            // return -1;
        
        // List is empty
        if(head == -1)
            return -1;

        int32_t tmpIndex = head;
        for (uint32_t i = 0; i < index; i++)
        {
            tmpIndex = nodes[tmpIndex].nextNode;
            // End of list
            if(tmpIndex == -1)
                return -1;            
        }
        
        ptr = &nodes[tmpIndex].data;

        return 0;
    }

    int valueByExpression(int (*expression)(T&, void*), T*& ptr, void* argsToExpression)
    {
        if(head == -1)
            return -1;

        int32_t tmpIndex = head;
        if(expression(nodes[tmpIndex].data, argsToExpression) == 0)
        {
            ptr = &nodes[tmpIndex].data;
            return 0;
        }
        while(tmpIndex != -1)
        {
            tmpIndex = nodes[tmpIndex].nextNode;
            if(expression(nodes[tmpIndex].data, argsToExpression) == 0)
            {
                ptr = &nodes[tmpIndex].data;
                return 0;
            }
        }

        return -1;
    }

    int size()
    {
        
        int32_t tmpHead = head;
        uint32_t counter = 0;
        
        while (tmpHead != -1)
        {
            counter++;
            tmpHead = nodes[tmpHead].nextNode;
        }
        
        return counter;
    }
};
