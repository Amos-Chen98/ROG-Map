#include <rog_map/local_mapper.h>
#include <rog_map_msgs/validation.h>
#include <gtest/gtest.h>

namespace {
using namespace rog_map;
PointCloud points(const Vec3f& p, int count=3) {
  PointCloud cloud;
  for (int i=0;i<count;++i) { PclPoint v{}; v.x=p.x();v.y=p.y();v.z=p.z();cloud.push_back(v); }
  return cloud;
}
rog_map_msgs::LocalMap get(const LocalMapper& mapper) {
  rog_map_msgs::LocalMap m; mapper.snapshot(m); m.header.frame_id="world";m.epoch=1;m.version=1;
  rog_map_msgs::validate(m); return m;
}
bool occupied(const rog_map_msgs::LocalMap& m, const Vec3f& p) {
  const Vec3i id = ((p-Vec3f(m.origin.x,m.origin.y,m.origin.z))/m.resolution).array().floor().cast<int>();
  if ((id.array()<0).any() || id.x()>=int(m.size[0]) || id.y()>=int(m.size[1]) || id.z()>=int(m.size[2])) return false;
  return rog_map_msgs::bit(m.occupied_bits, id.x()+m.size[0]*(id.y()+m.size[1]*id.z()));
}
void checkInflation(const rog_map_msgs::LocalMap& m) {
  std::vector<uint8_t> expected(m.inflated_bits.size(),0);
  int r=m.inflation_steps;
  for (int z=0;z<int(m.size[2]);++z) for(int y=0;y<int(m.size[1]);++y) for(int x=0;x<int(m.size[0]);++x) {
    size_t i=x+m.size[0]*(y+m.size[1]*z);
    if (!rog_map_msgs::bit(m.occupied_bits,i)) continue;
    for(int dz=-r;dz<=r;++dz) for(int dy=-r;dy<=r;++dy) for(int dx=-r;dx<=r;++dx) {
      int xx=x+dx, yy=y+dy, zz=z+dz;
      if(xx<0||yy<0||zz<0||xx>=int(m.size[0])||yy>=int(m.size[1])||zz>=int(m.size[2])) continue;
      rog_map_msgs::setBit(expected,xx+m.size[0]*(yy+m.size[1]*zz));
    }
  }
  EXPECT_EQ(m.inflated_bits,expected);
}
TEST(LocalMapper, FusionClearAndEmptyScans) {
  LocalMapper mapper(ros::NodeHandle("~"));
  Vec3f origin(.05,.05,.05), obstacle(.55,.05,.05);
  mapper.integrate({},origin); auto initial=get(mapper);
  EXPECT_EQ(std::count(initial.occupied_bits.begin(),initial.occupied_bits.end(),0),initial.occupied_bits.size());
  mapper.integrate(points(obstacle),origin); auto occupied_map=get(mapper);
  EXPECT_TRUE(occupied(occupied_map,obstacle));checkInflation(occupied_map);
  mapper.integrate({},origin); EXPECT_EQ(get(mapper).occupied_bits,occupied_map.occupied_bits);
  mapper.integrate(points(Vec3f(.95,.05,.05),10),origin);
  EXPECT_FALSE(occupied(get(mapper),obstacle));checkInflation(get(mapper));
  mapper.clear();checkInflation(get(mapper)); EXPECT_FALSE(occupied(get(mapper),obstacle));
}
TEST(LocalMapper, SlidingAndTeleportRemoveOldHistory) {
  LocalMapper mapper(ros::NodeHandle("~"));
  for(int i=0;i<24;++i) {
    Vec3f sensor(.05+.17*i,-.55+.08*i,.05);
    mapper.integrate(points(sensor+Vec3f(.45,.25,.15)),sensor);
    checkInflation(get(mapper));
  }
  mapper.integrate({},Vec3f(30,30,30));
  auto m=get(mapper);
  EXPECT_EQ(std::count(m.occupied_bits.begin(),m.occupied_bits.end(),0),m.occupied_bits.size());
  checkInflation(m);
  mapper.integrate({},Vec3f(.05,.05,.05));checkInflation(get(mapper));
}
TEST(LocalMapper, BoundaryAndInvalidRay) {
  LocalMapper mapper(ros::NodeHandle("~"));Vec3f sensor(.05,.05,.05);
  mapper.integrate({},sensor);auto m=get(mapper);
  Vec3f low(m.origin.x+.025,m.origin.y+.025,m.origin.z+.025);
  Vec3f high(m.origin.x+m.size[0]*m.resolution-.025,m.origin.y+.55,m.origin.z+.55);
  mapper.integrate(points(low),sensor);mapper.integrate(points(high),sensor);
  EXPECT_TRUE(occupied(get(mapper),low));EXPECT_TRUE(occupied(get(mapper),high));checkInflation(get(mapper));
  auto before=get(mapper);
  auto bad=points(sensor);bad.push_back(PclPoint{});bad.back().x=std::numeric_limits<float>::quiet_NaN();
  mapper.integrate(bad,sensor);EXPECT_EQ(get(mapper).occupied_bits,before.occupied_bits);
  EXPECT_THROW(mapper.integrate({},Vec3f(NAN,0,0)),std::invalid_argument);
}
}
int main(int argc,char**argv) {
  ros::init(argc,argv,"local_mapper_test");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
