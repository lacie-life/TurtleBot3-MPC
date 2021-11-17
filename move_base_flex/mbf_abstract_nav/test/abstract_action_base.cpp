#include <gmock/gmock.h>
#include <gtest/gtest.h>

// dummy message
#include <mbf_msgs/GetPathAction.h>
#include <mbf_utility/robot_information.h>

#include <mbf_abstract_nav/abstract_action_base.hpp>
#include <mbf_abstract_nav/abstract_execution_base.h>

using namespace mbf_abstract_nav;

// mocked version of an execution
struct MockedExecution : public AbstractExecutionBase {
  typedef boost::shared_ptr<MockedExecution> Ptr;

  MockedExecution() : AbstractExecutionBase("mocked_execution") {}

  MOCK_METHOD0(cancel, bool());

protected:
  MOCK_METHOD0(run, void());
};

using testing::Test;

// fixture with access to the AbstracActionBase's internals
struct AbstractActionBaseFixture
    : public AbstractActionBase<mbf_msgs::GetPathAction, MockedExecution>,
      public Test {
  // required members for the c'tor
  TF tf_;
  std::string test_name;
  mbf_utility::RobotInformation ri;

  AbstractActionBaseFixture()
      : test_name("action_base"),
        ri(tf_, "global_frame", "local_frame", ros::Duration(0)),
        AbstractActionBase(test_name, ri)
  {
  }

  void runImpl(GoalHandle &goal_handle, MockedExecution &execution) {}
};

TEST_F(AbstractActionBaseFixture, thread_stop)
{
  unsigned char slot = 1;
  concurrency_slots_[slot].execution.reset(new MockedExecution());
  concurrency_slots_[slot].thread_ptr =
      threads_.create_thread(boost::bind(&AbstractActionBaseFixture::run, this,
                                         boost::ref(concurrency_slots_[slot])));
}

using testing::Return;

TEST_F(AbstractActionBaseFixture, cancelAll)
{
  // spawn a bunch of threads
  for (unsigned char slot = 0; slot != 10; ++slot) {
    concurrency_slots_[slot].execution.reset(new MockedExecution());
    // set the expectation
    EXPECT_CALL(*concurrency_slots_[slot].execution, cancel())
        .WillRepeatedly(Return(true));

    // set the in_use flag --> this should turn to false
    concurrency_slots_[slot].in_use = true;
    concurrency_slots_[slot].thread_ptr = threads_.create_thread(
        boost::bind(&AbstractActionBaseFixture::run, this,
                    boost::ref(concurrency_slots_[slot])));
  }

  // cancel all of slots
  cancelAll();

  // check the result
  for (ConcurrencyMap::iterator slot = concurrency_slots_.begin();
       slot != concurrency_slots_.end(); ++slot)
    ASSERT_FALSE(slot->second.in_use);
}

int main(int argc, char **argv)
{
  // we need this only for kinetic and lunar distros
  ros::init(argc, argv, "abstract_action_base");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}