# -*- python -*-
# WAF tool to process XSL files.

from waflib import Utils, Errors
from waflib.Task import Task
from waflib.Node import split_path
from waflib.TaskGen import feature, before_method, extension
from waflib.Configure import conf
from waflib.Utils import to_list

def configure(conf):
        conf.find_program('xsltproc',var='XSLTPROC')

@conf
def xslt(self,target,xsl,xml,ext_out='.h',xslt_options='',stdout=False,name=None):
    self.phase('early0')

    if not isinstance(target,self.path.__class__):
            target=self.path.find_or_declare(target)
    
    if stdout:
            rule = '${XSLTPROC} %s ${SRC} > ${TGT}' % xslt_options
    else:
            rule = '${XSLTPROC} --stringparam filename \'${TGT}\' %s ${SRC}' % xslt_options

    self(
            name = name if name else target.name,
            source = [xsl, xml],
            target = target,
            rule   = rule,
            color  = 'BLUE'
            )
    self.phase_pop()

@conf
def gencli(self, xml, h=True, c=True,
           c_xsl='scripts/gen_cli_src.xsl',
           h_xsl='scripts/gen_cli_hdr.xsl',
           ext_out='.h', name=None):

        if not isinstance(xml,self.path.__class__):
                xml_node=self.path.find_or_declare(xml)
        else:
                xml_node = xml

        if xml_node is None:
                raise Errors.WafError('Could not find XML file %r'%xml)

        c_xsl_node = self.path.find_resource(c_xsl)
        if not c_xsl_node:
                c_xsl_node = self.root.find_resource(self.top_dir+'/'+c_xsl)
        if c_xsl_node is None:
                raise Errors.WafError('Could not find XSL file %r'%c_xsl)

        h_xsl_node = self.path.find_resource(h_xsl) or self.root.find_resource(h_xsl)
        if not h_xsl_node:
                h_xsl_node = self.root.find_resource(self.top_dir+'/'+h_xsl)
        if h_xsl_node is None:
                raise Errors.WafError('Could not find XSL file %r'%h_xsl)

        if c == True:
                c = xml_node.change_ext('.c')
        if h == True:
                h = xml_node.change_ext('.h')

        if h:
                self.xslt(target=h, xml = xml, xsl = h_xsl_node)
        if c:
                self.xslt(target=c, xml = xml, xsl = c_xsl_node)

@conf
def genpredef(self, xml, c=None, h=None,
              c_xsl='usr/lib/libdpi/predefined/gen_predef_metadata_c.xsl',
              h_xsl='usr/lib/libdpi/predefined/gen_predef_metadata_h.xsl',
              ext_out='.h', name=None, xslt_options='', stdout=False,
              c_prefix='vs_dpi_predef_', c_suffix='_metadata',
              h_prefix='include/vs_dpi_predef_', h_suffix='_metadata',
              ):
        """
        Generate predefined metadata headers and sources.

        :type xml: string or Node
        :param xml: XML file to use as source
        :param c: Name for generated C file, 
                  None - for default name derived from XML file
                  False - disable C file generations
        :param h: Name for generated H file
                  None - use default name derived from XML file
        :param c_xsl: XSL file to generate C files
        :param h_xsl: XSL file to generate H files
        :param xslt_options: additional options to pass to XSLTPROC
        :param stdout: True if generated code is expected to be printed on stdout
        """

        if not isinstance(xml,self.path.__class__):
                xml_node=self.path.find_resource(xml)
                if not xml_node:
                        xml_node=self.root.find_resource(self.top_dir+'/'+xml)
                if xml_node is None:
                        raise Errors.WafError('Could not find XML file %r'%xml)

                xml = xml_node

        # Figure out app name based on XML file name by stripping [_md].xml
        app = xml.name.replace('.xml','')
        app = app.replace('_md', '')

        c_xsl_node = self.path.find_resource(c_xsl)
        if not c_xsl_node:
                c_xsl_node = self.root.find_resource(self.top_dir+'/'+c_xsl)

        if c_xsl_node is None:
                raise Errors.WafError('Could not find XSL file %r'%c_xsl)

        h_xsl_node = self.path.find_resource(h_xsl)
        if not h_xsl_node:
                h_xsl_node = self.root.find_resource(self.top_dir+'/'+h_xsl)

        if h_xsl_node is None:
                raise Errors.WafError('Could not find XSL file %r'%h_xsl)

        if c != False:
                c_name = c if c else '%s%s%s.c' % (c_prefix, app, c_suffix)
                c_node = self.path.find_or_declare(c_name)

                self.xslt(target=c_node, xml=xml, xsl=c_xsl_node, 
                          xslt_options=xslt_options, stdout=stdout)

        if h != False:
                h_name = h if h else '%s%s%s.h' % (h_prefix, app, h_suffix)
                h_node = self.path.find_or_declare(h_name)

                self.xslt(target=h_node, xml = xml, xsl = h_xsl_node, 
                          xslt_options=xslt_options, stdout=stdout)
