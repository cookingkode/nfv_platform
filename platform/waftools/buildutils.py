# -*- python -*-
# WAF build helpers

# From the WAF book
from waflib.TaskGen import feature, before_method, after_method
from waflib.Configure import conf

@feature('*') 
@before_method('process_rule')
def post_the_other(self):
    deps = getattr(self, 'depends_on', []) 
    for name in self.to_list(deps):
        other = self.bld.get_tgen_by_name(name) 
        other.post() 

@feature('host')
@before_method('process_source')
def set_host_env(self, *k, **kw):
    self.env = self.bld.env.hostenv

@conf
def host_program(self, *k, **kw):
    kw['env'] = self.env.hostenv.derive()
    return self.program(*k, **kw)

@conf
def phase(self, phase=None):
    if phase == None:
        self.group_stack.pop()
        phase = self.group_stack[-1]
    else:
        self.group_stack.append(phase)
    self.set_group(phase)

@conf
def phase_pop(self):
    self.phase(None)

# convert include paths into relative ones so sandbox can be copied
# somewhere else (like target VM for debugging)
@feature('c','cxx','d','asm','fc','includes')
@after_method('apply_incpaths')
def filter_incpaths(self):
        lst = self.includes_nodes
        blddir = self.bld.bldnode
        self.env['INCPATHS']=[x.path_from(blddir) for x in lst]


# Remove -Werror from CFLAGS and CXXFLAGS if we're doing static
# analysis run and define VS_DEBUG so asserts give analyzer hints
@feature('c','cxx','d','asm','fc','includes')
@after_method('propagate_uselib_vars')
def filter_cflags(self):
        if not self.env.STATIC_ANALYSIS:
            return
        self.env.CFLAGS = filter(lambda a: a != '-Werror', self.env.CFLAGS)
        self.env.CXXFLAGS = filter(lambda a: a != '-Werror', self.env.CXXFLAGS)
        self.env.append_unique('DEFINES',['STATIC_ANALYSIS','VS_DEBUG'])

