#include <stdio.h>

#include <chrono>
#include <boost/filesystem.hpp>

#include "testimagedata.h"
#include "testimageloader.h"

#include <visionresult.pb.h>

namespace vlog = protobuf::test::vision;
namespace bf  = boost::filesystem;
using namespace htwk;
using namespace std;
using namespace std::chrono;

void createProtobufData(VisionPtr seg, vlog::VisionFrame& frame);
void saveResultFile(string filename, char* buffer, int size);

int main(int argc, char** argv)
{
    std::list<TestImageData> testData;
    TestImageLoader imageLoader;

    if(argc == 0)
    {
        printf("Please specify a directory for which we create test data.");
    }

    imageLoader.processDirectory(testData, bf::path(argv[1]), false);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    for(const TestImageData& image : testData)
    {
        string resultFilename = image.filename + ".result";
        printf("Create result file: %s\n", resultFilename.c_str());

        vlog::VisionFrame frame;
        createProtobufData(image.visionResult, frame);

        int desiredSize = frame.ByteSize();
        char* buffer = new char[desiredSize];

        if(buffer == NULL) {
            fprintf(stderr, "Error malloc buffer for image %s %d\n", __FILE__, __LINE__);
            return -1;
        }

        if(frame.SerializeToArray(buffer, desiredSize) == false) {
            fprintf(stderr, "Error serializing protobuf in image %s %d\n", __FILE__, __LINE__);
            delete[] buffer;
            return -1;
        }

        saveResultFile(resultFilename, buffer, desiredSize);

        delete[] buffer;
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << time_span.count() * 1000 << "ms\n\n";

    return 0;
}

void saveResultFile(string filename, char* buffer, int size)
{
    FILE* fp = fopen(filename.c_str(), "w");
    if(fp == NULL)
    {
        fprintf(stderr, "Error opening result file %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        return;
    }

    if(fwrite(&size, sizeof(size), 1, fp) != 1)
    {
        fprintf(stderr, "Error writing size to file %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        return;
    }

    if(fwrite(buffer, size, 1, fp) != 1)
    {
        fprintf(stderr, "Error writing data to file %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        return;
    }

    fclose(fp);
}


static const int width = 640;


template <class T>
inline void toProtobuf(const htwk::point_2d& p, T* w)
{
    w->set_x(p.x);
    w->set_y(p.y);
}

void createProtobufData(VisionPtr seg, vlog::VisionFrame& frame)
{
    /* ----------------------------------------- */
    color sfcolor = seg->fieldColorDetector->getColor();
    vlog::Color* fcolor = frame.mutable_fieldcolor();
    fcolor->set_cb(sfcolor.cb);
    fcolor->set_cr(sfcolor.cr);
    fcolor->set_cy(sfcolor.cy);

    /* ----------------------------------------- */
    vlog::FieldBorder* fBorder = frame.mutable_fieldborder();
    const int* tmpFBorder = seg->fieldDetector->getConvexFieldBorder();
    for(int x = 0; x < width; x++)
    {
        fBorder->add_ycoord(tmpFBorder[x]);
    }

    /* ----------------------------------------- */
    vector<LineSegment*>* lineSegments = seg->regionClassifier->lineSegments;
    for(vector<LineSegment*>::const_iterator i = lineSegments->begin(); i!=lineSegments->end(); ++i) {
        vlog::LineSegment* lSegment = frame.add_linesegments();
        lSegment->set_x((*i)->x);
        lSegment->set_y((*i)->y);
        lSegment->set_vx((*i)->vx);
        lSegment->set_vy((*i)->vy);
        lSegment->set_id((*i)->id);

        if((*i)->link == NULL) {
            lSegment->set_linkx(65535);
            lSegment->set_linky(65535);
        } else {
            lSegment->set_linkx((*i)->link->x);
            lSegment->set_linky((*i)->link->y);
        }

    }

    /* ----------------------------------------- */
    const vector<GoalPost>& goalPosts = seg->goalDetector->getGoalPosts();
    for(vector<GoalPost>::const_iterator i = goalPosts.begin(); i!=goalPosts.end(); ++i) {
        vlog::GoalPost* gp = frame.add_goalposts();

        gp->set_width(i->width);
        gp->set_probability(i->probability);

        toProtobuf(i->basePoint, gp->mutable_basepoint());
        toProtobuf(i->upperPoint, gp->mutable_upperpoint());

        vlog::Color* c = gp->mutable_color();
        c->set_cy(i->color.cy);
        c->set_cb(i->color.cb);
        c->set_cr(i->color.cr);
    }

    /* ----------------------------------------- */
    const vector<LineGroup>& linesList = seg->lineDetector->linesList;
    for(vector<LineGroup>::const_iterator i = linesList.begin(); i != linesList.end(); ++i) {
        vlog::Line* line = frame.add_lines();
        vlog::LineEdge* lineEdge = line->add_edges();
        lineEdge->set_px1(i->lines[0].px1);
        lineEdge->set_px2(i->lines[0].px2);
        lineEdge->set_py1(i->lines[0].py1);
        lineEdge->set_py2(i->lines[0].py2);
        lineEdge->set_nx(i->lines[0].nx);
        lineEdge->set_ny(i->lines[0].ny);
        lineEdge->set_d(i->lines[0].d);
        lineEdge->set_x(i->lines[0].x);
        lineEdge->set_y(i->lines[0].y);
        lineEdge->set_id(i->lines[0].id);
        lineEdge->set_matchcnt(i->lines[0].matchCnt);
        lineEdge->set_straight(i->lines[0].straight);
        lineEdge->set_valid(i->lines[0].valid);

        lineEdge = line->add_edges();
        lineEdge->set_px1(i->lines[1].px1);
        lineEdge->set_px2(i->lines[1].px2);
        lineEdge->set_py1(i->lines[1].py1);
        lineEdge->set_py2(i->lines[1].py2);
        lineEdge->set_nx(i->lines[1].nx);
        lineEdge->set_ny(i->lines[1].ny);
        lineEdge->set_d(i->lines[1].d);
        lineEdge->set_x(i->lines[1].x);
        lineEdge->set_y(i->lines[1].y);
        lineEdge->set_id(i->lines[1].id);
        lineEdge->set_matchcnt(i->lines[1].matchCnt);
        lineEdge->set_straight(i->lines[1].straight);
        lineEdge->set_valid(i->lines[1].valid);
    }

    /* ----------------------------------------- */
    Ellipse& el = seg->ellipseFitter->getEllipse();
    vlog::Ellipse* vel = frame.mutable_ellipse();
    vel->set_a (el.a);
    vel->set_a1(el.a1);
    vel->set_b (el.b);
    vel->set_b1(el.b1);
    vel->set_c (el.c);
    vel->set_c1(el.c1);
    vel->set_d (el.d);
    vel->set_d1(el.d1);
    vel->set_e (el.e);
    vel->set_e1(el.e1);
    vel->set_f (el.f);
    vel->set_f1(el.f1);
    vel->set_ta(el.ta);
    vel->set_tb(el.tb);
    vel->set_brennpunkt(el.brennpunkt);

    /* ----------------------------------------- */
    vlog::Ball* vball = frame.mutable_ball();
    Ball ball = seg->ballDetector->getBall();
    vball->set_x(ball.x);
    vball->set_y(ball.y);
    vball->set_radius(ball.radius);
    vball->set_found(ball.found);

    const vector<LineCross>& crossings = seg->lineDetector->crossings;
    for(vector<LineCross>::const_iterator i = crossings.begin(); i!=crossings.end(); ++i) {
        vlog::LineCross* cross = frame.add_linecrosses();
        cross->set_px(i->px);
        cross->set_py(i->py);
        cross->set_vx(i->vx);
        cross->set_vy(i->vy);
    }

    /* ----------------------------------------- */
    vlog::Point2D* vfeet = frame.mutable_feet();
    vfeet->set_x(seg->feetDetector->getBase().x);
    vfeet->set_y(seg->feetDetector->getBase().y);

    /* ----------------------------------------- */
    vector<RobotClassifierResult> robots = seg->getRobotClassifierResult();
    for(vector<RobotClassifierResult>::const_iterator i = robots.begin(); i!=robots.end(); ++i) {
        vlog::Robot* lRobot = frame.add_robot();
        lRobot->set_xleft(((*i).rect.xLeft));
        lRobot->set_xright(((*i).rect.xRight));
        lRobot->set_ytop((*i).rect.yTop);
        lRobot->set_ybottom((*i).rect.yBottom);
        lRobot->set_isblue((*i).isBlueJersey);
        lRobot->set_confidence((*i).detectionProbability);
    }
}
