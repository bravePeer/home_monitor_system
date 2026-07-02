#include <gtest/gtest.h>
#include "utilities/ring_buffer.hpp"


TEST(RingBufferTest, FullBuffer)
{
    RingBuffer<int, 10> ring;
    for (int i = 0; i < 10; i++)
    {
        ASSERT_EQ(0, ring.push(i));
    }
    int a = 16;
    ASSERT_EQ(-1, ring.push(a));
    ASSERT_EQ(10, ring.size());
}

TEST(RingBufferTest, EmptyBuffer)
{
    RingBuffer<int, 10> ring;
    int a = 0;
    ASSERT_EQ(-1, ring.pop(a));
    ASSERT_EQ(0, ring.size());
}

TEST(RingBufferTest, Buffer)
{
    RingBuffer<int, 10> ring;
    int a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        ASSERT_EQ(0, ring.push(a));
    }
    ASSERT_EQ(10, ring.size());

    a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        int tmp = 0;
        ASSERT_EQ(0, ring.pop(tmp));
        ASSERT_EQ(a, tmp);
    }

    ASSERT_EQ(0, ring.size());
}

TEST(RingBufferTest, ClearBuffer)
{
    RingBuffer<int, 10> ring;
    for (int i = 0; i < 5; i++)
    {
        ring.push(i);
    }

    ASSERT_EQ(ring.size(), 5);
    
    ring.clear();
    ASSERT_EQ(ring.size(), 0);
    
    int a = -1;
    ASSERT_EQ(ring.pop(a), -1);
}

// int main( int argc, char **argv) {
//     UNITY_BEGIN();
//     RUN_TEST(testFullBuffer);
//     RUN_TEST(testEmptyBuffer);
//     RUN_TEST(testBuffer);
//     UNITY_END();
// }