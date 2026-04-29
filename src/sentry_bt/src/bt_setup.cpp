#include "bt_setup.hpp"

#include <tinyxml2.h>

#include <sstream>
#include <stdexcept>

#include "bt_nodes_basic.hpp"
#include "bt_nodes_executor.hpp"
#include "bt_nodes_guard.hpp"
#include "bt_nodes_rule.hpp"
#include "bt_nodes_tactical.hpp"

namespace
{
std::runtime_error XMLExportError(const std::string& message,
                                  const std::filesystem::path& path)
{
    std::ostringstream oss;
    oss << message << ": " << path;
    return std::runtime_error(oss.str());
}
}  // 匿名命名空间

void RegisterAllNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    RegisterBasicNodes(factory, ctx);
    RegisterGuardNodes(factory, ctx);
    RegisterTacticalNodes(factory, ctx);
    RegisterRuleNodes(factory, ctx);
    RegisterExecutorNodes(factory, ctx);
}

void ExportTreeModelXML(const BT::BehaviorTreeFactory& factory,
                        const std::filesystem::path& template_path,
                        const std::filesystem::path& output_path,
                        bool include_builtin_models)
{
    tinyxml2::XMLDocument tree_doc;
    if (tree_doc.LoadFile(template_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        throw XMLExportError("Failed to load BT template XML", template_path);
    }

    auto* root = tree_doc.RootElement();
    if (!root || std::string(root->Name()) != "root")
    {
        throw XMLExportError("BT template XML root element is invalid", template_path);
    }

    for (auto* model = root->FirstChildElement("TreeNodesModel"); model != nullptr;)
    {
        auto* next = model->NextSiblingElement("TreeNodesModel");
        root->DeleteChild(model);
        model = next;
    }

    const std::string model_xml = BT::writeTreeNodesModelXML(factory, include_builtin_models);

    tinyxml2::XMLDocument model_doc;
    if (model_doc.Parse(model_xml.c_str()) != tinyxml2::XML_SUCCESS)
    {
        throw std::runtime_error("Failed to parse generated TreeNodesModel XML");
    }

    auto* model_root = model_doc.RootElement();
    auto* model_element = model_root ? model_root->FirstChildElement("TreeNodesModel") : nullptr;
    if (!model_element)
    {
        throw std::runtime_error("Generated XML does not contain a <TreeNodesModel>");
    }

    root->InsertEndChild(model_element->DeepClone(&tree_doc));

    if (tree_doc.SaveFile(output_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        throw XMLExportError("Failed to save BT XML with TreeNodesModel", output_path);
    }
}
