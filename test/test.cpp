#include <sstream>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/exception-factory.h>
#include <dynamic-graph/pool.h>

//Dynamic graph tutorial
#include <dynamic_graph_fcl/DynamicGraphFCL.h>

/*
struct MyEntity : public dynamicgraph::Entity
{
  static const std::string CLASS_NAME;

  MyEntity (const std::string& name)
    : Entity (name)
  {}

  virtual void display (std::ostream& os) const
  {
    os << "Hello! My name is " << getName () << " !" << std::endl;
  }

  virtual const std::string& getClassName () const
  {
    return CLASS_NAME;
  }
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (MyEntity, "MyEntity");
*/
int main(void){

    using namespace dynamicgraph;
    using     namespace FCL;

    DynamicGraphFCL* fcl =
            dynamic_cast<DynamicGraphFCL*>(
                dynamicgraph::FactoryStorage::getInstance()->
                newEntity("DynamicGraphFCL", "DynamicGraphFCL"));


    dynamicgraph::Signal<double, int> *aOut;
    aOut = dynamic_cast<dynamicgraph::Signal<double, int> *>(&(fcl->getSignal("aOUT")));

    dynamicgraph::Signal<double, int> *bOut;
    bOut = dynamic_cast<dynamicgraph::Signal<double, int> *>(&(fcl->getSignal("bOUT")));

    dynamicgraph::Signal<double, int>  *zIN = NULL;
    zIN = dynamic_cast<dynamicgraph::Signal<double, int> *>(&(fcl->getSignal("zIN")));
    zIN->setConstant(2.);
    zIN->recompute(0);
//    zIN->plug(bOut);
    std::cout << "dynamic graph plugged" << std::endl;
//    stateIn =  dynamic_cast<dynamicgraph::Signal<double, int> *>(&(fcl->getSignal("aOut")));

    bOut->recompute(0);
    std::cout << "recompute done" << std::endl;
    const double& bOUT1 = bOut->accessCopy();

    std::cout << "aOut signal read " << aOut->accessCopy() << std::endl;
    std::cout << "bOut signal read " << bOut->accessCopy() << std::endl;
    std::cout << "bOut1 signal read " << bOUT1 << std::endl;
    dynamicgraph::PoolStorage::getInstance()->writeGraph("test");

}

