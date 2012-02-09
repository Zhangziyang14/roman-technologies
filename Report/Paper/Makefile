# zonder extensie
ROOT = project_eva
BIB = biblio

$(ROOT).pdf: *.tex
	latex -interaction=nonstopmode $(ROOT).tex
	dvips -o $(ROOT).ps $(ROOT).dvi
	ps2pdf $(ROOT).ps

clean:
	rm *.dvi *.aux *.ps *.toc *.out *.log
