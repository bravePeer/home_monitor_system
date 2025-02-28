#include <gtest/gtest.h>
#include "utilities/ring_buffer.hpp"


TEST(RingBufferTest, FullBuffer)
{
    RingBuffer<int, 10> ring;
    for (int i = 0; i < 10; i++)
    {
        ASSERT_EQ(0, ring.pushData(i));
    }
    int a = 16;
    ASSERT_EQ(-1, ring.pushData(a));
    ASSERT_EQ(10, ring.getDataCount());
}

TEST(RingBufferTest, EmptyBuffer)
{
    RingBuffer<int, 10> ring;
    int a = 0;
    ASSERT_EQ(-1, ring.popData(a));
    ASSERT_EQ(0, ring.getDataCount());
}

TEST(RingBufferTest, Buffer)
{
    RingBuffer<int, 10> ring;
    int a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        ASSERT_EQ(0, ring.pushData(a));
    }
    ASSERT_EQ(10, ring.getDataCount());

    a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        int tmp = 0;
        ASSERT_EQ(0, ring.popData(tmp));
        ASSERT_EQ(a, tmp);
    }

    ASSERT_EQ(0, ring.getDataCount());
}

// int main( int argc, char **argv) {
//     UNITY_BEGIN();
//     RUN_TEST(testFullBuffer);
//     RUN_TEST(testEmptyBuffer);
//     RUN_TEST(testBuffer);
//     UNITY_END();
// }