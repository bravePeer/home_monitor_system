#include <gtest/gtest.h>
#include <utilities/list.hpp>

TEST(ListTest, CreateList)
{
    List<int, 20> list;
}

TEST(ListTest, AddToList)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        ASSERT_EQ(list.add(i), 0);

    // List is full
    int val = 21;
    ASSERT_EQ(list.add(val), -1);

    int* ptr = nullptr;
    int tmpRes = 19;
    for (int i = 0; i < 20; i++)
    {
        ASSERT_NE(list.valueAt(i, ptr), -1);
        ASSERT_EQ(*ptr, tmpRes--);
    }
}

TEST(ListTest, GetFromEmptyList)
{
    List<int, 20> list;

    int* ptr = nullptr;
    int val = -1;
    ASSERT_EQ(list.valueAt(0, ptr), -1);
    ASSERT_EQ(list.valueAt(15, ptr), -1);
    ASSERT_EQ(list.valueAt(100, ptr), -1);
}

TEST(ListTest, RemoveFromListIfEmpty)
{
    List<int, 20> list;

    ASSERT_EQ(list.remove(0), -1);
    ASSERT_EQ(list.remove(15), -1);
    ASSERT_EQ(list.remove(32), -1);
}

TEST(ListTest, RemoveFirstFromList)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        ASSERT_EQ(list.add(i), 0);

    int* ptr = nullptr;
    ASSERT_EQ(list.valueAt(0, ptr), 0);
    ASSERT_EQ(*ptr, 19);
    ASSERT_EQ(list.remove(0), 0);
    ASSERT_EQ(list.valueAt(0, ptr), 0);
    ASSERT_EQ(*ptr, 18);
}

TEST(ListTest, RemoveLastFromList)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        list.add(i);

    int* ptr = nullptr;
    int val = -1;
    ASSERT_EQ(list.valueAt(19, ptr), 0);
    ASSERT_EQ(*ptr, 0);
    ASSERT_EQ(list.remove(19), 0);
    int ret = list.valueAt(19, ptr);
    ASSERT_EQ(ret, -1);
}


TEST(ListTest, RemoveFromList)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        list.add(i);

    list.remove(1);
    list.remove(1);
    list.remove(1);

    int* ptr = nullptr;
    int val = -1;
    int tmp = 19;
    ASSERT_EQ(list.valueAt(0, ptr), 0);
    ASSERT_EQ(*ptr, tmp--);
    tmp -= 3;

    for (int i = 1; i < 20 - 3; i++)
    {
        val = -1;
        ASSERT_EQ(list.valueAt(i, ptr), 0);
        ASSERT_EQ(*ptr, tmp--);
    }
    ASSERT_EQ(list.valueAt(17, ptr), -1);
    ASSERT_EQ(list.valueAt(18, ptr), -1);
    ASSERT_EQ(list.valueAt(19, ptr), -1);
}

TEST(ListTest, FindValueByExpression)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        list.add(i);

    int* ptr;
    int ret = list.valueByExpression([](int& arg, void* s)->int {
        if(arg == 10)
            return 0;
        return -1;
    }, ptr, nullptr);

    ASSERT_EQ(ret, 0);
    ASSERT_EQ(*ptr, 10);

    ret = list.valueByExpression([](int& arg, void* s)->int {
        if(arg == 100)
            return 0;
        return -1;
    }, ptr, nullptr);
    ASSERT_EQ(ret, -1);
}