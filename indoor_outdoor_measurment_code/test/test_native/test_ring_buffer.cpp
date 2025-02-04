#include <unity.h>
#include "utilities/ring_buffer.hpp"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void testFullBuffer()
{
    RingBuffer<int, 10> ring;
    for (int i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL(0, ring.pushData(i));
    }
    int a = 16;
    TEST_ASSERT_EQUAL(-1, ring.pushData(a));
    TEST_ASSERT_EQUAL(10, ring.getDataCount());
}

void testEmptyBuffer()
{
    RingBuffer<int, 10> ring;
    int a = 0;
    TEST_ASSERT_EQUAL(-1, ring.popData(a));
    TEST_ASSERT_EQUAL(0, ring.getDataCount());
}

void testBuffer()
{
    RingBuffer<int, 10> ring;
    int a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        TEST_ASSERT_EQUAL(0, ring.pushData(a));
    }
    TEST_ASSERT_EQUAL(10, ring.getDataCount());

    a = 3;
    for (int i = 0; i < 10; i++)
    {
        a *= i;
        int tmp = 0;
        TEST_ASSERT_EQUAL(0, ring.popData(tmp));
        TEST_ASSERT_EQUAL(a, tmp);
    }

    TEST_ASSERT_EQUAL(0, ring.getDataCount());
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testFullBuffer);
    RUN_TEST(testEmptyBuffer);
    RUN_TEST(testBuffer);
    UNITY_END();
}