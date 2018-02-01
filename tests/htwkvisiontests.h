#ifndef HTWKVISIONTESTS_H
#define HTWKVISIONTESTS_H

#include <list>
#include <mutex>

#include <cppunit/extensions/HelperMacros.h>

#include "testimagedata.h"
#include "testimageloader.h"

class HTWKVisionTests : public CppUnit::TestFixture
{
CPPUNIT_TEST_SUITE(HTWKVisionTests);
CPPUNIT_TEST(testBalls);
CPPUNIT_TEST(testGoalPosts);
CPPUNIT_TEST(testFieldBorders);
CPPUNIT_TEST(testLineSegments);
CPPUNIT_TEST(testRobotHypothesis);
CPPUNIT_TEST(testFeet);
CPPUNIT_TEST(testLineCrossings);
CPPUNIT_TEST(testLineGroups);
CPPUNIT_TEST(testEllipse);
CPPUNIT_TEST_SUITE_END();

protected:
    static std::list<TestImageData> testData;

    static bool wasLoaded;
    static std::mutex mutex;

    TestImageLoader imageLoader;

public:
    HTWKVisionTests();
    virtual ~HTWKVisionTests();

    virtual void setUp();
    virtual void tearDown();

    void testBalls();
    void testGoalPosts();
    void testFieldBorders();
    void testLineSegments();
    void testRobotHypothesis();
    void testFeet();
    void testLineCrossings();
    void testLineGroups();
    void testEllipse();
};

#endif // HTWKVISIONTESTS_H
