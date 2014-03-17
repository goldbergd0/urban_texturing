// Dan Goldberg

#include "element.h"

Element::Element()
  : elname_(""), properties_(std::list<Property>()), numpr_(0),numel(0){}

void Element::addProperty(const Property& p){
  properties_.push_front(p);
  numpr_++;
  numel_+=p.getDataSize();
}
void Element::popProperty(){
  Property back = properties_.back();
  properties_.pop_back();
  return back;
}

bool Element::writeHeader(FILE* file)const{
  if (!file) return false;
  fprintf(file, "element %s %u\n", elname_.c_str(), numel_);
  std::list::iterator it;
  for (it = properties_.begin(); it != properties_.end(); it++){
    it.writePropertyHeader(file);
  }
  return true;
}

bool Element::writeData(FILE* file)const{
  if (!file) return false;
  fprintf(file, 
  return true;
}



