#include <filesystem>
#include <iostream>
#include <memory>

#include "bt_setup.hpp"

int main()
{
    try
    {
        BT::BehaviorTreeFactory factory;
        auto ctx = std::make_shared<RobotContext>();
        RegisterAllNodes(factory, ctx);

        const auto template_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.template.xml";
        const auto output_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.xml";

        ExportTreeModelXML(factory, template_path, output_path, false);

        std::cout << "Exported BT XML with TreeNodesModel to " << output_path << std::endl;
        std::cout << "Registered custom nodes: " << factory.manifests().size()
                  << ", builtin nodes: " << factory.builtinNodes().size() << std::endl;
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_bt_export_tree failed: " << ex.what() << std::endl;
        return 1;
    }
}
