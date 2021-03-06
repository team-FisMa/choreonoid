/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ScriptItem.h"
#include "ItemManager.h"
#include "ItemTreeView.h"
#include "MenuManager.h"
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void ScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<ScriptItem, AbstractTextItem>();

    ItemTreeView::customizeContextMenu<ScriptItem>(
        [](ScriptItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Execute"))->sigTriggered().connect([item](){ item->execute(); });
            menuManager.addItem(_("Terminate"))->sigTriggered().connect([item](){ item->terminate(); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


ScriptItem::ScriptItem()
{

}


ScriptItem::ScriptItem(const ScriptItem& org)
    : AbstractTextItem(org)
{

}


ScriptItem::~ScriptItem()
{

}


const std::string& ScriptItem::textFilename() const
{
    return scriptFilename();
}


std::string ScriptItem::identityName() const
{
    const string& name_ = name();
    stdx::filesystem::path path(fromUTF8(scriptFilename()));
    const string fname = toUTF8(path.filename().string());
    if(name_.empty()){
        return fname;
    }
    if(fname == name_){
        return name_;
    } else {
        return name_ + " (" + fname + ")";
    }
}


bool ScriptItem::isBackgroundMode() const
{
    return false;
}


bool ScriptItem::isRunning() const
{
    return false;
}


bool ScriptItem::executeCode(const char* code)
{
    return false;
}


bool ScriptItem::waitToFinish(double timeout)
{
    return false;
}


std::string ScriptItem::resultString() const
{
    return string();
}
