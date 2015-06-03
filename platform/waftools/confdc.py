# -*- python -*-
# WAF build helper for CONFDC operations

#Task generator
import re
from waflib import Task, Logs
from waflib.TaskGen import extension, declare_chain, feature, after_method, before_method
from waflib.Configure import conf
from waflib.Utils import to_list

def configure(conf):
    conf.confdc_common_flags()

################################################################
# Simple YANG parser to find included files
################################################################
INC_REGEX="""(?:^|['">]\s*;)\s*include\s+(.+?)\s*;"""
re_inc=re.compile(INC_REGEX,re.I)

class yang_parser(object):
    def __init__(self,incpaths):
        self.seen=[]
        self.nodes=[]
        self.names=[]
        self.incpaths=incpaths

    def find_deps(self,node):
        txt=node.read()
        incs=[]
        uses=[]
        mods=[]
        for line in txt.splitlines():
                m=re_inc.search(line)
                if m:
                        incs.append(m.group(1))
        return(incs)

    def start(self,node):
        self.thisdir=node.parent
        self.waiting=[node]
        while self.waiting:
                nd=self.waiting.pop(0)
                self.iter(nd)

    def iter(self,node):
        path=node.abspath()
        incs=self.find_deps(node)
        for x in incs:
                if x in self.seen:
                        continue
                self.seen.append(x)
                self.tryfind_header(node, x)

    def tryfind_header(self, node, filename):
        filename = filename + ".yang"
        found=None
        for n in self.incpaths + [node.parent]:
            found=n.find_resource(filename)
            if found:
                self.nodes.append(found)
                self.waiting.append(found)
                break
        if not found:
            if not filename in self.names:
                self.names.append(filename)


################################################################
# YANG-> {FXS, TAGS.H} rules
################################################################
@conf
def yang(self, *k, **kw):
    self.phase('early0')
    features = getattr(kw,'features',[])
    for f in ['confdc','use']:
        if f not in features:
            features.append(f)

    kw['features'] = features;
    x = self(*k, **kw)
    self.phase_pop()
    return x

@feature('confdc')
@after_method('propagate_uselib_vars','process_source')
def apply_confdc_options(self):
    omap = getattr(self,'option_map',{})

    # Process Include-like options
    # These add paths to source and build dirs.
    for o in ['yang_includes','mib_includes', 'fxs_includes'] :
        eo = o.upper()
        lst=self.to_incnodes(self.to_list(getattr(self,o,[])))
        # confdc errors out if include directories do not exist.
        # Filter out directories that do not exist in source tree
        lst = [x for x in lst if (x.is_src() and x.find_dir('.')) or x.is_bld()]
        setattr(self, o+'_nodes', lst)
        self.env[eo]=[x.path_from(self.bld.bldnode) for x in lst]

    # Process node-like includes
    for o in ['yang_annotations', 'yang_deviations', 'mib_annotations', 'fxs_includefile'] :
        eo = o.upper()
        lst = self.to_list(getattr(self,o,[]))
        lst = self.to_nodes(lst)
        setattr(self, o+'_nodes', lst)
        self.env[eo]=[x.path_from(self.bld.bldnode) for x in lst]

    # Process regular option list
    for o in ['yang_errors','yang_warnings', 'yang_features', 'confdc_options',
              'mib_includefile'] :
        eo = o.upper()
        lst=self.to_list(getattr(self,o,[]))
        self.env[eo]=lst

@conf
def confdc_common_flags(conf):
    v=conf.env
    v.YANG_ANN_ST  = '-a %s'
    v.YANG_INC_ST  = '--yangpath %s'
    v.YANG_DEV_ST  = '--deviation %s'
    v.YANG_ERR_ST  = '-E %s'
    v.YANG_WARN_ST = '-W %s'
    v.YANG_FEAT_ST = '-F %s'
    v.MIB_INC_ST  = '-I %s'
    v.MIB_INCBIN_ST  = '--include-file %s'
    v.MIB_ANN_ST  = '--mib-annotation %s'
    v.CONFDC_FXSDEP_ST = '-f %s'
    v.CONFDC_OPTIONS=[]

def yang_scan(self):
    tmp=yang_parser(self.generator.yang_includes_nodes)
    tmp.task=self
    tmp.start(self.inputs[0])
    if Logs.verbose:
        Logs.debug('deps: deps for %r: %r; unresolved %r'%(self.inputs,tmp.nodes,tmp.names))
    return(tmp.nodes,tmp.names)

confdc_common_options = [
    '${CONFDC_OPTIONS}',
    '${CONFDC_FXSDEP_ST:FXS_INCLUDES}'
    '${CONFDC_FXSDEP_ST:FXS_INCLUDEFILE}'
    '${YANG_INC_ST:YANG_INCLUDES}',
]

confdc_yang_options = confdc_common_options + [
    '${YANG_ANN_ST:YANG_ANNOTATIONS}',
    '${YANG_DEV_ST:YANG_DEVIATIONS}',
    '${YANG_ERR_ST:YANG_ERRORS}',
    '${YANG_WARN_ST:YANG_WARNINGS}',
    '${YANG_FEAT_ST:YANG_FEATURES}',
    ]

confdc_mib_options = confdc_common_options + [
    '${MIB_INC_ST:MIB_INCLUDES}',
    '${MIB_INCBIN_ST:MIB_INCLUDEFILE}',
    '${MIB_ANN_ST:MIB_ANNOTATIONS}',
]

class yangcompile(Task.Task):
    color='BLUE'
    run_str = '${CONFDC} -c %s -o ${TGT} -- ${SRC}' % ' '.join(confdc_yang_options)
    shell = True
    scan = yang_scan
    ext_out = ['.fxs']

class emith(Task.Task):
    color='BOLD'
    run_str = '${CONFDC} --emit-h ${TGT} ${SRC} '
    ext_out = ['.h']

class emitmib(Task.Task):
    color='BLUE'
    run_str = '${CONFDC} -c %s -o ${TGT} -- ${SRC}' % ' '.join(confdc_yang_options)
    shell = True
    scan = yang_scan
    ext_out = ['.fxs']

@extension('.yang')
def handle_yang(self,node):
    fxsdir = getattr(self,'fxs_dir',None)
    if fxsdir:
        n = '%s/%s' % (fxsdir, node.change_ext('.fxs'))
        fxsnode = self.path.find_or_declare(n)
    else:
        fxsnode = node.change_ext('.fxs')
    
    tsk=self.create_task('yangcompile', node, fxsnode)
    self.source.append(fxsnode)

@extension('.fxs')
def handle_fxs(self,node):
    hext = getattr(self,'h_ext',None)
    hext = hext if hext else '-tags.h'
    
    hdir = getattr(self,'h_dir',None)
    if hdir:
        n = '%s/%s' % (hdir, node.change_ext(hext))
        hnode = self.path.find_or_declare(n)
    else:
        hnode = node.change_ext(hext)
    tsk=self.create_task('emith', node, hnode)
                     
################################################################
# {MIB,FXS} -> BIN rules
################################################################

@conf
def bin(self, target, fxs, mib, ext_in = ['.mib','.fxs'], ext_out = '.bin', **kw):
    rule = '${CONFDC} -c ${SRC} %s -o ${TGT} ' % ' '.join(to_list(confdc_mib_options))
    self(
        group = 'early0',
        features = 'use confdc',
        source = [mib, fxs],
        target = target,
        color  = 'YELLOW',
        shell  = True,
        rule   = rule,
        ext_in = ext_in,
        ext_out = ext_out,
        **kw
        )

################################################################
# FXS -> MIB rules
################################################################

@conf
def mib(self, target, fxs, oid, **kw):
    topname = "%s" % fxs.change_ext('')
    topname = topname.replace('-','')
    topname += "MIB"

    xopt = "--top %s --generate-oids --oid %s" % (topname, oid)
    rule = '${CONFDC} --emit-mib ${TGT} %s -- ${SRC} ' % (xopt + (' '.join(confdc_mib_options)))

    self(
        group = 'early0',
        features = 'use confdc',
        source = fxs,
        target = target,
        color  = 'YELLOW',
        shell  = True,
        rule   = rule,
        ext_in = '.fxs',
        ext_out = '.mib',
        **kw
        )

################################################################
# CLI -> CCL compilation rule 
################################################################
declare_chain(
    name = 'clicompile',
    rule = '${CONFDC} -c ${SRC} -o ${TGT}', 
    ext_in = '.cli',
    ext_out = '.ccl',
    reentrant = False,
    #scan = yang_scan
    )
