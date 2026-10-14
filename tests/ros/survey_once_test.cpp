#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>
#include <stdexcept>

int calls = 0;
BT::NodeStatus survey_result = BT::NodeStatus::RUNNING;
class FakeSurvey : public BT::ActionNodeBase {
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {++calls; return survey_result;}
  void halt() override {setStatus(BT::NodeStatus::IDLE);}
};
void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char ** argv) {
  try {
    require(argc == 2, "Pass SurveyOnce plugin library");
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(argv[1]);
    factory.registerNodeType<FakeSurvey>("FakeSurvey");
    const char * xml = R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <SurveyOnce><FakeSurvey/></SurveyOnce></BehaviorTree></root>)";
    auto tree = factory.createTreeFromText(xml);
    require(tree.tickRoot() == BT::NodeStatus::RUNNING, "Survey did not wait for completion");
    tree.haltTree();
    survey_result = BT::NodeStatus::FAILURE;
    require(tree.tickRoot() == BT::NodeStatus::FAILURE && calls == 2, "Cancelled survey skipped");
    tree.haltTree();
    survey_result = BT::NodeStatus::SUCCESS;
    require(tree.tickRoot() == BT::NodeStatus::SUCCESS && calls == 3, "Failed survey was not retried");
    tree.haltTree(); // Same navigator starts the next navigation goal.
    survey_result = BT::NodeStatus::FAILURE;
    require(tree.tickRoot() == BT::NodeStatus::SUCCESS && calls == 3, "Completed survey repeated");
    auto fresh = factory.createTreeFromText(xml);
    require(fresh.tickRoot() == BT::NodeStatus::FAILURE && calls == 4, "Fresh navigator skipped survey");
    std::cout << "PASS: survey completion, cancellation, failure, goal reuse, fresh navigator\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
