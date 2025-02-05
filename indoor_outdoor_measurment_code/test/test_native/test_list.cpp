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


    int tmpRes = 19;
    for (int i = 0; i < 20; i++)
    {
        val = -1;
        ASSERT_NE(list.valueAt(i, val), -1);
        ASSERT_EQ(val, tmpRes--);
    }
}

TEST(ListTest, GetFromEmptyList)
{
    List<int, 20> list;

    int val = -1;
    ASSERT_EQ(list.valueAt(0, val), -1);
    ASSERT_EQ(list.valueAt(15, val), -1);
    ASSERT_EQ(list.valueAt(100, val), -1);
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

    int val = -1;
    ASSERT_EQ(list.valueAt(0, val), 0);
    ASSERT_EQ(val, 19);
    ASSERT_EQ(list.remove(0), 0);
    ASSERT_EQ(list.valueAt(0, val), 0);
    ASSERT_EQ(val, 18);
}

TEST(ListTest, RemoveLastFromList)
{
    List<int, 20> list;

    for (int i = 0; i < 20; i++)
        list.add(i);

    int val = -1;
    ASSERT_EQ(list.valueAt(19, val), 0);
    ASSERT_EQ(val, 0);
    ASSERT_EQ(list.remove(19), 0);
    int ret = list.valueAt(19, val);
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

    int val = -1;
    int tmp = 19;
    ASSERT_EQ(list.valueAt(0, val), 0);
    ASSERT_EQ(val, tmp--);
    tmp -= 3;

    for (int i = 1; i < 20 - 3; i++)
    {
        val = -1;
        ASSERT_EQ(list.valueAt(i, val), 0);
        ASSERT_EQ(val, tmp--);
    }
    ASSERT_EQ(list.valueAt(17, val), -1);
    ASSERT_EQ(list.valueAt(18, val), -1);
    ASSERT_EQ(list.valueAt(19, val), -1);
}