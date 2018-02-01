#include "htwkvisiontests.h"

#include <stdio.h>
#include <boost/filesystem.hpp>

#include "testutils.h"

namespace bf  = boost::filesystem;
using namespace htwk;

const int width = 640;

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION(HTWKVisionTests);

bool HTWKVisionTests::wasLoaded;
std::mutex HTWKVisionTests::mutex;
std::list<TestImageData> HTWKVisionTests::testData;


HTWKVisionTests::HTWKVisionTests()
{
    std::lock_guard<std::mutex> lock(mutex);

    if(wasLoaded == false)
    {
        imageLoader.processDirectory(testData, bf::path(TestUtils::testImagePath), true);
        wasLoaded = true;
    }
}

HTWKVisionTests::~HTWKVisionTests()
{

}

void HTWKVisionTests::setUp()
{
}

void HTWKVisionTests::tearDown()
{

}

void HTWKVisionTests::testBalls()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        auto& a = test.groundTruthData.ball();
        Ball b = test.visionResult->ballDetector->getBall();

        if(a.x() != b.x
            || a.y() != b.y
            || a.radius() != b.radius
            || a.found() != b.found)
        {
            printf("Ball missmatch in file: %s\n", test.filename.c_str());
            result = false;
        }
    }

    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testGoalPosts()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        const auto& goalPosts = test.visionResult->goalDetector->getGoalPosts();

        if(goalPosts.size() != test.groundTruthData.goalposts_size())
        {
            printf("Number of goal post missmatch in file %s\n", test.filename.c_str());
            result = false;
        }

        for(int i = 0; i < goalPosts.size(); i++)
        {
            auto& a = test.groundTruthData.goalposts(i);
            const GoalPost& b = goalPosts[i];

            if(a.width() != b.width
                || a.probability() != b.probability
                || a.color().cy() != b.color.cy
                || a.color().cb() != b.color.cb
                || a.color().cr() != b.color.cr
                || a.basepoint().x() != b.basePoint.x
                || a.basepoint().y() != b.basePoint.y
                || a.upperpoint().x() != b.upperPoint.x
                || a.upperpoint().y() != b.upperPoint.y)
            {
                printf("Goal posts missmatch in file: %s\n", test.filename.c_str());
                result = false;
            }
        }
    }

    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testFieldBorders()
{
    bool result = true;
    for(TestImageData& test : testData)
    {
        auto& pBorder = test.groundTruthData.fieldborder();
        auto* iBorder = test.visionResult->fieldDetector->getConvexFieldBorder();

        for(int i = 0; i < width; i++)
        {
            if(pBorder.ycoord(i) != iBorder[i])
            {
                printf("Different Border for index %d: %d vs %d file %s\n", i , pBorder.ycoord(i), iBorder[i], test.filename.c_str());
                result = false;
            }
        }
    }
    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testLineSegments()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        const int pCount = test.groundTruthData.linesegments_size();
        const int iCount = test.visionResult->regionClassifier->lineSegments->size();
        if(pCount != iCount)
        {
            printf("Number of line segments miss match %d vs %d file %s\n", pCount, iCount, test.filename.c_str());
            result = false;
            continue;
        }

        for(int i = 0; i < pCount; i++)
        {
            LineSegment* a = (*test.visionResult->regionClassifier->lineSegments)[i];
            auto& b = test.groundTruthData.linesegments(i);

            if(a->x != b.x()
                || a->y != b.y()
                || a->vx != b.vx()
                || a->vy != b.vy()
                || a->id != b.id())
            {
                printf("LineSegments differ file %s\n", test.filename.c_str());
                result = false;
            }

            int linkX = 65535;
            int linkY = 65535;

            if(a->link != NULL) {
                linkX = a->link->x;
                linkY = a->link->y;
            }

            if(b.linkx() != linkX || b.linky() != linkY)
            {
                printf("LineSegments links differ file %s\n", test.filename.c_str());
                result = false;
            }
        }
    }
    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testFeet()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        if(test.groundTruthData.feet().x() != test.visionResult->feetDetector->getBase().x
            || test.groundTruthData.feet().y() != test.visionResult->feetDetector->getBase().y)
        {
            printf("Feet position differ file %s\n", test.filename.c_str());
            result = false;
        }
    }

    CPPUNIT_ASSERT(result);
}



void HTWKVisionTests::testRobotHypothesis()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        std::vector<RobotClassifierResult> r = test.visionResult->getRobotClassifierResult();

        if(r.size() != test.groundTruthData.robot_size())
        {
            printf("Number of robots miss match %zu vs %d file %s\n", r.size(), test.groundTruthData.robot_size(), test.filename.c_str());
            result = false;
            continue;
        }

        for(int i = 0; i < r.size(); i++)
        {
            RobotClassifierResult a = r[i];
            auto b = test.groundTruthData.robot(i);

            if(a.isBlueJersey != b.isblue()
                || a.detectionProbability != b.confidence()
                || a.rect.xLeft != b.xleft()
                || a.rect.xRight != b.xright()
                || a.rect.yTop != b.ytop()
                || a.rect.yBottom != b.ybottom())
            {
                printf("Robots differ file %s\n", test.filename.c_str());
                result = false;
            }
        }
    }

    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testLineCrossings()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        auto& c = test.visionResult->lineDetector->crossings;

        if(c.size() != test.groundTruthData.linecrosses_size())
        {
            printf("Number of line crossings missmatch %zu vs %d file %s\n", c.size(), test.groundTruthData.linecrosses_size(), test.filename.c_str());
            result = false;
            continue;
        }

        for(int i = 0; i < c.size(); i++)
        {
            auto a = c[i];
            auto b = test.groundTruthData.linecrosses(i);

            if(a.px != b.px()
                || a.py != b.py()
                || a.vx != b.vx()
                || a.vy != b.vy())
            {
                printf("Line crosses differ in file %s\n", test.filename.c_str());
                result = false;
            }
        }

    }
    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testLineGroups()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        const std::vector<LineGroup>& l = test.visionResult->lineDetector->linesList;
        int pCount = test.groundTruthData.lines_size();

        if(l.size() != pCount)
        {
            printf("Number of line groups missmatch %zu vs %d file %s\n", l.size(), pCount, test.filename.c_str());
            result = false;
            continue;
        }

        std::vector<LineEdge> cEdges;
        std::vector<protobuf::test::vision::LineEdge> pEdges;

        for(int i = 0; i < l.size(); i++)
        {
            cEdges.push_back(l[i].lines[0]);
            cEdges.push_back(l[i].lines[1]);
            pEdges.push_back(test.groundTruthData.lines(i).edges(0));
            pEdges.push_back(test.groundTruthData.lines(i).edges(1));
        }

        std::stable_sort(cEdges.begin(), cEdges.end(), [](const LineEdge& a, const LineEdge& b) -> bool { return a.px1 > b.px1; });
        std::stable_sort(cEdges.begin(), cEdges.end(), [](const LineEdge& a, const LineEdge& b) -> bool { return a.px2 > b.px2; });
        std::stable_sort(cEdges.begin(), cEdges.end(), [](const LineEdge& a, const LineEdge& b) -> bool { return a.py1 > b.py1; });
        std::stable_sort(cEdges.begin(), cEdges.end(), [](const LineEdge& a, const LineEdge& b) -> bool { return a.py2 > b.py2; });

        std::stable_sort(pEdges.begin(), pEdges.end(), [](const protobuf::test::vision::LineEdge& a, const protobuf::test::vision::LineEdge& b) -> bool { return a.px1() > b.px1(); });
        std::stable_sort(pEdges.begin(), pEdges.end(), [](const protobuf::test::vision::LineEdge& a, const protobuf::test::vision::LineEdge& b) -> bool { return a.px2() > b.px2(); });
        std::stable_sort(pEdges.begin(), pEdges.end(), [](const protobuf::test::vision::LineEdge& a, const protobuf::test::vision::LineEdge& b) -> bool { return a.py1() > b.py1(); });
        std::stable_sort(pEdges.begin(), pEdges.end(), [](const protobuf::test::vision::LineEdge& a, const protobuf::test::vision::LineEdge& b) -> bool { return a.py2() > b.py2(); });

        auto compare = [&test, &result](const LineEdge& a, const protobuf::test::vision::LineEdge& b)
        {
            if(a.px1 != b.px1()
                || a.px2 != b.px2()
                || a.py1 != b.py1()
                || a.py2 != b.py2()
                || a.nx != b.nx()
                || a.ny != b.ny()
                || a.x != b.x()
                || a.y != b.y()
//                || a.id != b.id()
//                || a.matchCnt != b.matchcnt()
                || a.straight != b.straight()
                || a.valid != b.valid())
            {
                printf("Line group differ in file %s\n", test.filename.c_str());
                printf("Line live vs proto\n");
                printf("px1: %f vs %f\n", a.px1, b.px1());
                printf("px2: %f vs %f\n", a.px2, b.px2());
                printf("py1: %f vs %f\n", a.py1, b.py1());
                printf("py2: %f vs %f\n", a.py2, b.py2());
                printf("nx: %f vs %f\n", a.nx, b.nx());
                printf("ny: %f vs %f\n", a.ny, b.ny());
                printf("x: %f vs %f\n", a.x, b.x());
                printf("y: %f vs %f\n", a.y, b.y());
                printf("id: %d vs %d\n", a.id, b.id());
                printf("matchCnt: %d vs %d\n", a.matchCnt, b.matchcnt());
                printf("straight: %d vs %d\n", a.straight, b.straight());
                printf("valid: %d vs %d\n", a.valid, b.valid());
                result = false;
            }
        };

        for(int i = 0; i < cEdges.size(); i++)
        {
            compare(cEdges[i], pEdges[i]);
        }

    }
    CPPUNIT_ASSERT(result);
}

void HTWKVisionTests::testEllipse()
{
    bool result = true;

    for(TestImageData& test : testData)
    {
        Ellipse& a = test.visionResult->ellipseFitter->getEllipse();
        auto b = test.groundTruthData.ellipse();

        if(a.a != b.a()
            || a.a1 != b.a1()
            || a.b != b.b()
            || a.b1 != b.b1()
            || a.c != b.c()
            || a.c1 != b.c1()
            || a.d != b.d()
            || a.d1 != b.d1()
            || a.e != b.e()
            || a.e1 != b.e1()
            || a.f != b.f()
            || a.f1 != b.f1()
            || a.ta != b.ta()
            || a.tb != b.tb()
            || a.brennpunkt != b.brennpunkt())
        {
            printf("Ellipse differs in file %s\n", test.filename.c_str());
            result = false;
        }
    }
    CPPUNIT_ASSERT(result);
}

